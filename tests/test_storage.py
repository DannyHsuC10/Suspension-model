import copy
import json
import pickle
from pathlib import Path
import tempfile
import unittest

import numpy as np

from suspension_dynamics.project import default_project, save_project, load_project, estimate_damping, parameters_for_analysis
from suspension_dynamics.engine import simulate, steady_response
from suspension_dynamics.project_import import read_import_project
from suspension_dynamics.storage import save_result, load_result, write_csv, write_report, import_legacy, fingerprint
from suspension_dynamics.charts import envelope


class StorageTests(unittest.TestCase):
    @staticmethod
    def legacy_wheel_parameters(full):
        full = full.copy()
        for component in ('corner', 'tire'):
            for axle in ('f', 'r'):
                for kind in ('k', 'c', 'mr'):
                    value = full.pop(f'{component}_{axle}_{kind}')
                    for side in ('L', 'R'):
                        full[f'{component}_{axle.upper()}{side}_{kind}'] = value
        return full

    def setUp(self):
        self.temp_base=Path(__file__).resolve().parents[1]/'.test-tmp';self.temp_base.mkdir(exist_ok=True)
        self.temp=tempfile.TemporaryDirectory(dir=self.temp_base);self.root=Path(self.temp.name)
        p=default_project();p['analysis']['full'].update(duration=.2,start=.01,dt=.003)
        self.result=simulate(p,'full')

    def tearDown(self):
        self.assertTrue(self.root.resolve().is_relative_to(self.temp_base.resolve()))
        self.temp.cleanup()

    def test_project_and_result_roundtrip_preserve_full_precision(self):
        r=self.result;path=self.root/'project.json';save_project(path,r.project)
        project = load_project(path)
        self.assertEqual(project["parameters"]["full"], r.project["parameters"]["full"])
        self.assertEqual(project["analysis"], default_project()["analysis"])
        path=self.root/'result.npz';save_result(path,r);loaded=load_result(path)
        self.assertEqual(loaded.info,r.info);self.assertEqual(loaded.project,r.project)
        for name in r.signals:np.testing.assert_array_equal(r.signals[name],loaded.signals[name])

    def test_legacy_mass_fields_migrate_for_projects_and_preserve_result_snapshots(self):
        project = copy.deepcopy(self.result.project)
        project['parameters']['quarter'] = parameters_for_analysis(project, 'quarter')
        project['analysis']['quarter'].pop('parameter_corner')
        full = project['parameters']['full']
        full.pop('mu_f')
        full.pop('mu_r')
        full.update(mu_FL=11., mu_FR=15., mu_RL=19., mu_RR=23.)
        path = self.root/'legacy.json'
        path.write_text(json.dumps(project), encoding="utf-8")
        updated = load_project(path)
        self.assertEqual(updated['analysis']['quarter']['parameter_corner'], 'FL')
        self.assertEqual(updated['parameters']['full']['mu_f'], 13.)
        self.assertEqual(updated['parameters']['full']['mu_r'], 21.)
        self.assertNotIn('mu_FL', updated['parameters']['full'])
        original = simulate(project, 'full')
        save_result(self.root/'legacy.npz', original)
        loaded = load_result(self.root/'legacy.npz')
        self.assertEqual(loaded.project, original.project)
        self.assertNotEqual(loaded.signals['Normal load FL'][0], loaded.signals['Normal load FR'][0])

    def test_compact_project_saves_one_parameter_group_and_result_keeps_resolved_fields(self):
        project = default_project()
        self.assertEqual(set(project['parameters']['quarter']), {'ks', 'cs', 'mr', 'zeta'})
        self.assertEqual(set(project['parameters']['LR']), {'k_frame', 'c_frame', 'm_frame'})
        self.assertEqual(project['parameters']['FR'], {})
        path = self.root/'compact.json'
        save_project(path, project)
        saved = json.loads(path.read_text(encoding="utf-8"))
        self.assertNotIn("analysis", saved)
        self.assertEqual(set(saved["parameters"]), {"full"})
        wheel_keys = {key for key in saved["parameters"]["full"] if key.startswith(("corner_", "tire_"))}
        self.assertEqual(wheel_keys, {f"{component}_{axle}_{kind}" for component in ("corner", "tire")
                                      for axle in ("f", "r") for kind in ("k", "c", "mr")})
        loaded_project = load_project(path)
        self.assertEqual(loaded_project["parameters"]["full"], project["parameters"]["full"])
        self.assertEqual(loaded_project["parameters"]["LR"], project["parameters"]["LR"])
        self.assertEqual(loaded_project["analysis"], project["analysis"])

        project['analysis']['FR'].update(duration=.2, start=.01, dt=.003, parameter_axle='rear')
        result = simulate(project, 'FR')
        path = self.root/'resolved.npz'
        save_result(path, result)
        loaded = load_result(path)
        self.assertEqual(loaded.project, result.project)
        self.assertEqual(loaded.project['parameters']['FR']['k_heave_f'],
                         2 * project['parameters']['full']['heave_r_k'])

    def test_sizing_export_derives_quarter_from_main_and_tender(self):
        project = default_project()
        full = self.legacy_wheel_parameters(project["parameters"]["full"]) | dict(
            heave_f_k=20_000, roll_f_k=60_000, corner_FL_mr=.75, corner_FR_mr=.85, zeta=.65,
            tire_FL_k=70_000., tire_FR_k=90_000., tire_RL_c=None, tire_RR_c=440.,
            tender_enabled=True, tender_stiffness_N_per_m=80_000,
            k_frame=900_000, c_frame=125, m_frame=50,
            dive_front=.2, lift_front=.15, squat_rear=.05, lift_rear=.4,
            drive_front=.5, brake_front=.75,
        )
        del full["anti_front"], full["anti_rear"]
        path = self.root / "initial_settings.json"
        path.write_text(json.dumps({
            "application": "VehicleInitialSizingLab-DynamicsExport", "schema_version": 1,
            "name": "Sizing", "parameters": {"full": full},
        }), encoding="utf-8")
        _, missing = read_import_project(path)
        self.assertEqual(missing, [])
        loaded = load_project(path)
        self.assertEqual(loaded["parameters"]["full"]["anti_front"], .2)
        self.assertEqual(loaded["parameters"]["full"]["anti_rear"], .4)
        self.assertNotIn("brake_front", loaded["parameters"]["full"])
        self.assertEqual(loaded["parameters"]["full"]["tire_f_k"], 80_000.)
        self.assertEqual(loaded["parameters"]["full"]["tire_r_c"], 440.)
        quarter = parameters_for_analysis(loaded, "quarter")
        self.assertEqual(quarter["ks"], 25_000)
        self.assertEqual(quarter["mr"], .8)
        self.assertEqual(quarter["zeta"], .65)
        self.assertAlmostEqual(quarter["cs"], 2 * .65 * np.sqrt(25_000 * quarter["ms"]) / .8)
        self.assertEqual(loaded["parameters"]["LR"], dict(k_frame=900_000, c_frame=125, m_frame=50))
        save_project(path, loaded)
        self.assertEqual(load_project(path), loaded)

    def test_categorized_sizing_export_loads_like_legacy(self):
        parameters = default_project()["parameters"]
        full = parameters["full"] | parameters["LR"] | {"zeta": 0.65, "c_frame": 125}
        vehicle_keys = ("m", "lf", "lr", "tf", "tr", "h_cg", "Ixx", "Iyy", "mu_f", "mu_r")
        geometry_keys = ("h_rc_f", "h_rc_r")
        vehicle_geometry = {"gravity": 9.81, "free_radius": 0.2, "clearance_steer_deg": 27.0}
        axle_geometry = {
            "front": {"fvsa": 0.97, "kpi_deg": 8.0, "caster_deg": 6.3},
            "rear": {"fvsa": 1.02, "kpi_deg": 9.0, "caster_deg": 0.0},
            "analysis": {"bump_mm": 25.0, "droop_mm": 20.0, "steer_deg": 27.0},
        }
        header = {"application": "VehicleInitialSizingLab-DynamicsExport", "name": "Sizing"}
        legacy = header | {
            "schema_version": 1, "parameters": {"full": full},
            "geometry": {"vehicle": vehicle_geometry} | axle_geometry,
        }
        categorized = header | {
            "schema_version": 2,
            "dynamics": {key: value for key, value in full.items() if key not in vehicle_keys + geometry_keys},
            "geometry": {key: full[key] for key in geometry_keys} | {
                "clearance_steer_deg": vehicle_geometry["clearance_steer_deg"],
            },
            "vehicle": {key: full[key] for key in vehicle_keys} | {
                "gravity": vehicle_geometry["gravity"], "free_radius": vehicle_geometry["free_radius"],
            },
        }
        nested = categorized | {"geometry": categorized["geometry"] | axle_geometry}
        flattened = categorized | {"geometry": categorized["geometry"] | {
            "fvsa_f": 0.97, "kpi_f_deg": 8.0, "caster_f_deg": 6.3,
            "fvsa_r": 1.02, "kpi_r_deg": 9.0, "caster_r_deg": 0.0,
            "bump_mm": 25.0, "droop_mm": 20.0, "steer_deg": 27.0,
        }}
        old_path = self.root / "legacy.json"
        new_path = self.root / "categorized.json"
        old_path.write_text(json.dumps(legacy), encoding="utf-8")
        expected = load_project(old_path)
        main_spring_export = flattened | {"schema_version": 3, "dynamics": flattened["dynamics"] | {
            key: 4 * full[key] for key in ("heave_f_k", "heave_r_k")
        }}
        for export in (nested, flattened):
            new_path.write_text(json.dumps(export), encoding="utf-8")
            _, missing = read_import_project(new_path)
            self.assertEqual(missing, [])
            self.assertEqual(load_project(new_path), expected)

        new_path.write_text(json.dumps(main_spring_export), encoding="utf-8")
        _, missing = read_import_project(new_path)
        self.assertEqual(missing, [])
        loaded = load_project(new_path)
        for key in ("heave_f_k", "heave_r_k"):
            self.assertEqual(loaded["parameters"]["full"][key], main_spring_export["dynamics"][key])
        self.assertEqual(loaded["parameters"]["quarter"], expected["parameters"]["quarter"])
        save_project(new_path, loaded)
        self.assertEqual(load_project(new_path), loaded)

    def test_archived_results_keep_the_original_wheel_parameters(self):
        result = copy.deepcopy(self.result)
        full = self.legacy_wheel_parameters(result.project["parameters"]["full"])
        result.project["parameters"]["full"] = full
        path = self.root / "four-corner.npz"
        save_result(path, result)
        loaded = load_result(path)
        self.assertEqual(loaded.project, result.project)
        self.assertEqual(steady_response(loaded), steady_response(self.result))

    def test_legacy_half_car_import_has_geometry_for_playback(self):
        path = self.root/'half.pkl'
        path.write_bytes(pickle.dumps({
            'data': {'t': np.array([0., .1]), 'front_zs': np.zeros(2)},
            'metadata': {'config': {'family': 'FR'}},
        }))
        result = import_legacy(path)
        parameters = result.project['parameters']['FR']
        self.assertGreater(parameters['lf'], 0.)
        self.assertGreater(parameters['lr'], 0.)

    def test_csv_and_report_include_units_and_input_snapshot(self):
        write_csv(self.root/'a.csv',self.result);write_report(self.root/'a.html',self.result)
        text=(self.root/'a.csv').read_text(encoding='utf-8-sig')
        self.assertIn('Normal load FL [N]',text);self.assertEqual(len(text.splitlines()),len(self.result.time)+1)
        report=(self.root/'a.html').read_text(encoding='utf-8')
        self.assertIn('180000',report);self.assertIn('body_twist_k',report)
        self.assertIn('<th>穩態平衡點</th>', report)
        self.assertIn('<th>2% 安定時間 (s)</th>', report)
        self.assertIn('<th>超越量 (%)</th>', report)

    def test_comparison_requires_identical_inputs_and_parameters(self):
        a=self.result;b=copy.deepcopy(a);b.project['analysis']['full']['model']='rigid'
        self.assertEqual(fingerprint(a),fingerprint(b))
        b.project['parameters']['full']['heave_f_k']+=1
        self.assertNotEqual(fingerprint(a),fingerprint(b))

    def test_envelope_retains_short_extreme(self):
        x=np.arange(10000);y=np.zeros(10000);y[7123]=123;y[13]=-55
        xd,yd=envelope(x,y,200)
        self.assertIn(123,yd);self.assertIn(-55,yd);self.assertLessEqual(len(xd),202)

    def test_damping_estimator_does_not_mutate_settings(self):
        p=parameters_for_analysis(default_project(), 'quarter');before=copy.deepcopy(p)
        estimated=estimate_damping('quarter',p)
        self.assertEqual(p,before)
        self.assertAlmostEqual(estimated['cs']*p['mr']**2,2*p['zeta']*np.sqrt(p['ks']*p['mr']**2*p['ms']))

    def test_all_supplied_legacy_archives_load_without_legacy_modules(self):
        source=Path(__file__).resolve().parents[2]/'new'/'Model_testing'/'batch_results'
        if not source.exists():self.skipTest('Original archives are optional for standalone distribution')
        files=list(source.glob('**/result.pkl'));self.assertTrue(files)
        for file in files:
            with self.subTest(file=file):
                r=import_legacy(file);self.assertTrue(r.legacy);self.assertIn('位移',r.groups)
                save_result(self.root/'old.npz',r);self.assertTrue(load_result(self.root/'old.npz').legacy)

    def test_pickle_executable_globals_are_rejected(self):
        class NotData:
            def __reduce__(self):return eval,('2+2',)
        path=self.root/'bad.pkl';path.write_bytes(pickle.dumps(NotData()))
        with self.assertRaises(ValueError):import_legacy(path)


if __name__=='__main__':unittest.main()
