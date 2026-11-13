import copy
import unittest

import numpy as np

from suspension_dynamics.catalog import MODELS, scenarios
from suspension_dynamics.engine import assemble, simulate, Road, Cancelled, G
from suspension_dynamics.project import default_project, parameters_for_analysis, validate
from suspension_dynamics.storage import statistics


class DynamicsTests(unittest.TestCase):
    def setUp(self):
        self.p=default_project()

    def run_case(self,family,model=None,**options):
        if model:self.p['analysis'][family]['model']=model
        self.p['analysis'][family].update(options)
        return simulate(self.p,family)

    def test_every_topology_and_supported_input_is_finite(self):
        for family,models in MODELS.items():
            for model in models:
                for scenario in scenarios(family,model):
                    with self.subTest(family=family,model=model,scenario=scenario):
                        r=self.run_case(family,model,scenario=scenario,duration=.36,start=.03,dt=.003)
                        self.assertTrue(all(np.isfinite(x).all() for x in r.signals.values()))
                        self.assertEqual(r.time[0],0);self.assertEqual(r.time[-1],.36)

    def test_zero_equilibrium_for_all_topologies(self):
        for family,models in MODELS.items():
            for model in models:
                r=self.run_case(family,model,duration=.12,start=.01,dt=.003,height=0,ax=0,ay=0)
                for name,value in r.signals.items():
                    if not name.startswith('Normal load'):np.testing.assert_array_equal(value,0,err_msg=family+model+name)

    def test_equivalent_single_matches_closed_form_step_response(self):
        r=self.run_case('quarter','single',scenario='step',start=0,duration=1.03,dt=.001)
        p=r.project['parameters']['quarter'];kw=p['ks']*p['mr']**2
        omega=np.sqrt((kw*p['kt']/(kw+p['kt']))/(p['ms']+p['mu']))
        exact=self.p['analysis']['quarter']['height']*(1-np.cos(omega*r.time))*1000
        np.testing.assert_allclose(r.signals['Body Q'],exact,atol=2e-6)

    def test_quarter_step_settles_at_road_height_and_static_load(self):
        r=self.run_case('quarter',scenario='step',start=.1,duration=5,dt=.002)
        self.assertAlmostEqual(r.signals['Body Q'][-1],10.,places=3)
        self.assertAlmostEqual(r.signals['Wheel Q'][-1],10.,places=3)
        p = r.project['parameters']['quarter']
        self.assertAlmostEqual(r.signals['Normal load Q'][-1], (p['ms'] + p['mu']) * G, places=2)

    def test_quarter_corner_matches_full_static_load_and_wheel_rates(self):
        full = self.p['parameters']['full']
        full.update(m=330., mu_f=13., mu_r=22., lf=.6, lr=.9,
                    tire_f_k=100000., tire_f_c=240., tire_f_mr=.8,
                    tire_r_k=110000., tire_r_c=400., tire_r_mr=1.2)
        vehicle = assemble('full', 'flex', full)
        np.testing.assert_allclose(vehicle.kt, [64000., 64000., 158400., 158400.])
        np.testing.assert_allclose(vehicle.ct, [153.6, 153.6, 576., 576.])
        np.testing.assert_array_equal(vehicle.masses[-4:], [13., 13., 22., 22.])
        self.assertAlmostEqual(vehicle.masses[0], 260.)
        for index, corner in enumerate(vehicle.labels):
            with self.subTest(corner=corner):
                r = self.run_case('quarter', parameter_corner=corner, duration=.2, start=.03, dt=.003)
                p = r.project['parameters']['quarter']
                self.assertEqual(r.info['parameter_corner'], corner)
                self.assertEqual(p['mu'], vehicle.masses[-4 + index])
                self.assertEqual((p['kt'], p['ct']), (vehicle.kt[index], vehicle.ct[index]))
                self.assertAlmostEqual(p['ms'], 78. if index < 2 else 52.)
                self.assertAlmostEqual(r.signals['Normal load Q'][0], vehicle.static_load[index])

    def test_step_metrics_use_theoretical_equilibrium_and_handle_both_signs(self):
        for height in (.01, -.01):
            with self.subTest(height=height):
                result = self.run_case('quarter', 'single', scenario='step', height=height,
                                       start=0., duration=.45, dt=.001)
                rows = {row[0]: row for row in statistics(result)}
                self.assertAlmostEqual(rows['Body Q'][2], height * 1000)
                self.assertEqual(rows['Body Q'][3], '未安定')
                self.assertAlmostEqual(rows['Body Q'][4], 100., delta=.01)
                self.assertEqual(rows['Road Q'][2:5], (height * 1000, 0., 0.))

    def test_bump_settles_back_to_zero_and_sine_has_no_fixed_equilibrium(self):
        options = dict(start=.1, duration=3., dt=.003)
        bump = self.run_case('quarter', 'quarter', scenario='bump', **options)
        body = next(row for row in statistics(bump) if row[0] == 'Body Q')
        self.assertEqual(body[2], 0.)
        self.assertGreaterEqual(body[3], self.p['analysis']['quarter']['period'])
        self.assertIsNone(body[4])
        sine = self.run_case('quarter', 'quarter', scenario='sine', **options)
        self.assertTrue(all(row[2:5] == (None, None, None) for row in statistics(sine)))

    def test_iso_road_class_psd_and_speed_to_distance_mapping(self):
        system = assemble('quarter', 'quarter', parameters_for_analysis(self.p, 'quarter'))
        options = dict(self.p['analysis']['quarter'], scenario='iso_a', speed=10., start=.1, seed=32)
        slow = Road(system, options)
        fast = Road(system, dict(options, speed=20.))
        class_b = Road(system, dict(options, scenario='iso_b'))
        class_c = Road(system, dict(options, scenario='iso_c'))
        other_seed = Road(system, dict(options, seed=33))
        np.testing.assert_array_equal(slow.at(0.)[:2], 0.)
        self.assertEqual(slow.at(.1)[0][0], 0.)
        for elapsed in (.013, .12, .63):
            t = options['start'] + elapsed
            height, velocity, _, _ = slow.at(t)
            fast_height, fast_velocity, _, _ = fast.at(options['start'] + elapsed / 2)
            np.testing.assert_allclose(fast_height, height, atol=1e-12)
            np.testing.assert_allclose(fast_velocity, 2 * velocity, atol=1e-12)
            np.testing.assert_allclose(class_b.at(t)[0], 2 * height, atol=1e-12)
            np.testing.assert_allclose(class_c.at(t)[0], 4 * height, atol=1e-12)
            self.assertNotAlmostEqual(other_seed.at(t)[0][0], height[0])
            derivative = (slow.at(t + 1e-6)[0] - slow.at(t - 1e-6)[0]) / 2e-6
            np.testing.assert_allclose(velocity, derivative, atol=1e-7)
        variance = 16e-6 * .1**2 * (1 / .011 - 1 / 2.83)
        self.assertAlmostEqual(np.var(slow.iso_profile.height), variance, delta=.02 * variance)

    def test_iso_road_rejects_coarse_output_sampling(self):
        with self.assertRaisesRegex(ValueError, '輸出步長'):
            self.run_case('quarter', scenario='iso_c', speed=30., dt=.01)

    def test_iso_road_rear_axle_samples_the_same_profile_after_wheelbase_delay(self):
        for family in ('FR', 'full'):
            with self.subTest(family=family):
                options = dict(self.p['analysis'][family], scenario='iso_b', target='all',
                               speed=20., start=.1, seed=32, rear_delay=True)
                params = parameters_for_analysis(self.p, family)
                system = assemble(family, options['model'], params)
                rear_start = options['start'] + (params['lf'] + params['lr']) / options['speed']
                axle_size = len(system.labels) // 2
                delayed = Road(system, options)
                simultaneous = Road(system, dict(options, rear_delay=False))
                self.assertIn(rear_start, delayed.events)
                for values in delayed.at(np.nextafter(rear_start, 0.))[:2]:
                    np.testing.assert_array_equal(values[axle_size:], 0.)
                for elapsed in (0., .047):
                    expected = simultaneous.at(options['start'] + elapsed)[:2]
                    actual = delayed.at(rear_start + elapsed)[:2]
                    for value, reference in zip(actual, expected):
                        np.testing.assert_allclose(value[axle_size:], reference[:axle_size], atol=1e-12)
                        np.testing.assert_array_equal(reference[:axle_size], reference[axle_size:])

    def test_mass_and_virtual_work_for_full_vehicle(self):
        for model in ('rigid','flex'):
            s=assemble('full',model,self.p['parameters']['full'])
            self.assertEqual(s.masses[0],261)
            rng=np.random.default_rng(24);q=rng.normal(size=s.dof);dq=rng.normal(size=s.dof)
            force=-(s.ks@s.D@q)
            tire_power=np.dot(dq[s.body_dof:]*s.kt,q[s.body_dof:])
            np.testing.assert_allclose(dq@(-s.K@q), (s.D@dq)@force-tire_power - (180000*(q[1]-q[2])*(dq[1]-dq[2]) if model=='flex' else 0),rtol=1e-12)
            self.assertAlmostEqual(s.B[0,1],self.p['parameters']['full']['tf']/2)

    def test_all_passive_systems_have_symmetric_nonnegative_energy_and_damping(self):
        rng=np.random.default_rng(10)
        for family,models in MODELS.items():
            for model in models:
                if model=='rollcenter_all':continue
                s=assemble(family,model,parameters_for_analysis(self.p, family))
                np.testing.assert_allclose(s.K,s.K.T,atol=1e-10)
                np.testing.assert_allclose(s.C,s.C.T,atol=1e-10)
                self.assertGreater(np.min(np.linalg.eigvalsh(s.K)),-1e-7)
                self.assertGreater(np.min(np.linalg.eigvalsh(s.C)),-1e-7)
                q,v=rng.normal(size=(2,s.dof))
                acceleration=(-s.K@q-s.C@v)/s.masses
                power=np.dot(v*s.masses,acceleration)+v@s.K@q
                self.assertAlmostEqual(power,-v@s.C@v,places=6)

    def test_undamped_full_free_vibration_conserves_energy(self):
        p=self.p['parameters']['full']
        for key in p:
            if key.endswith('_c'):p[key]=0.
        self.p['analysis']['full'].update(height=0.,duration=1.,dt=.001,start=.01)
        initial=np.zeros(16);initial[:4]=[.01,.003,-.003,.002]
        r=simulate(self.p,'full',initial=initial)
        energy=r.signals['Mechanical energy']
        self.assertLess(np.ptp(energy)/energy[0],2e-6)

    def test_full_left_right_mirror_reverses_roll_and_twist(self):
        a=self.run_case('full',target='FL',rear_delay=False,duration=.8,start=.1,dt=.002)
        b=self.run_case('full',target='FR',rear_delay=False,duration=.8,start=.1,dt=.002)
        for name in ('CG heave','Pitch'):np.testing.assert_allclose(a.signals[name],b.signals[name],atol=1e-10)
        for name in ('Roll front','Roll rear','Twist'):np.testing.assert_allclose(a.signals[name],-b.signals[name],atol=1e-10)
        np.testing.assert_allclose(a.signals['Normal load FL'],b.signals['Normal load FR'],atol=1e-8)

    def test_symmetric_road_does_not_induce_roll(self):
        r=self.run_case('full',target='all',duration=.8,dt=.002,start=.1)
        for name in ('Roll front','Roll rear','Twist'):self.assertLess(np.max(np.abs(r.signals[name])),1e-10)

    def test_full_steady_cornering_conserves_total_load_and_roll_moment(self):
        r=self.run_case('full',scenario='cornering',duration=4,dt=.002,start=.1)
        p=self.p['parameters']['full']
        loads={c:r.signals['Normal load '+c][-1] for c in ('FL','FR','RL','RR')}
        self.assertAlmostEqual(sum(loads.values()),p['m']*G,delta=.05)
        moment=(loads['FR']-loads['FL'])*p['tf']/2+(loads['RR']-loads['RL'])*p['tr']/2
        self.assertAlmostEqual(moment,p['m']*2*p['h_cg'],delta=.05)

    def test_full_steady_acceleration_conserves_pitch_moment(self):
        r=self.run_case('full',scenario='accelerate',duration=4,dt=.002,start=.1)
        p=self.p['parameters']['full'];s=assemble('full','flex',p)
        loads=np.array([r.signals['Normal load '+c][-1] for c in ('FL','FR','RL','RR')])-s.static_load
        moment=-np.dot([p['lf'],p['lf'],-p['lr'],-p['lr']],loads)
        self.assertAlmostEqual(moment,p['m']*2*p['h_cg'],delta=.1)
        self.assertAlmostEqual(sum(loads),0,delta=.05)

    def test_axle_anti_scales_signed_acceleration_independently(self):
        full = self.p['parameters']['full']
        full.update(lf=.6, lr=.9, anti_front=.3, anti_rear=.7)
        for family, model in (('full', 'rigid'), ('full', 'flex'), ('FR', 'half')):
            with self.subTest(family=family, model=model):
                p = parameters_for_analysis(self.p, family)
                system = assemble(family, model, p)
                zeros = np.zeros(len(system.labels))
                mass = full['m'] if family == 'full' else 2 * (p['ms'] + p['mu'])
                transfer = mass * 3 * full['h_cg'] / (full['lf'] + full['lr'])
                expected = transfer * np.array([.3, -.7])
                if family == 'full':
                    expected = np.repeat(expected / 2, 2)
                accelerating = system.input_force(zeros, zeros, 3., 0.)
                braking = system.input_force(zeros, zeros, -3., 0.)
                np.testing.assert_allclose(accelerating[system.body_dof:], expected)
                np.testing.assert_allclose(braking, -accelerating)

    def test_roll_center_changes_body_roll_without_changing_total_transfer(self):
        self.p['parameters']['full']['h_rc_f']=0
        a=self.run_case('LR',scenario='cornering',duration=3,dt=.002,start=.1)
        self.p['parameters']['full']['h_rc_f']=.2
        b=self.run_case('LR',scenario='cornering',duration=3,dt=.002,start=.1)
        self.assertLess(abs(b.signals['Roll'][-1]),abs(a.signals['Roll'][-1]))
        self.assertAlmostEqual(a.signals['Normal load R'][-1]-a.signals['Normal load L'][-1],b.signals['Normal load R'][-1]-b.signals['Normal load L'][-1],delta=.1)

    def test_lr_dual_mass_lateral_moment_transfers_load(self):
        result = self.run_case('LR', '2mass', scenario='cornering', ay=2., duration=3., start=.1, dt=.002)
        p = result.project['parameters']['LR']
        left = result.signals['Normal load L'][-1]
        right = result.signals['Normal load R'][-1]
        mass = 2 * (p['ms'] + p['mu'])
        self.assertAlmostEqual(left + right, mass * G, delta=.1)
        self.assertAlmostEqual((right - left) * p['track'] / 2, mass * 2 * p['h_cg'], delta=.1)
        self.assertGreater(result.signals['Roll (derived)'][-1], 0)
        equilibrium = {row[0]: row[2] for row in statistics(result)}
        transfer = (equilibrium['Normal load R'] - equilibrium['Normal load L']) * p['track'] / 2
        self.assertAlmostEqual(transfer, mass * 2 * p['h_cg'])

    def test_linked_lateral_model_uses_front_axle_element_rates_and_mass(self):
        full = self.p['parameters']['full']
        full.update(lf=.8, lr=1.2, mu_f=12., mu_r=19., tire_f_mr=1.2, tire_r_mr=.8,
                    heave_f_mr=.9, roll_f_mr=1.1, corner_f_k=2400., corner_f_c=100., corner_f_mr=1.3,
                    corner_r_k=5000., corner_r_c=200., corner_r_mr=.8)
        lateral = assemble('LR', 'half', parameters_for_analysis(self.p, 'LR'))
        vehicle = assemble('full', 'flex', full)
        np.testing.assert_allclose(vehicle.channel_k[-4:], [4056., 4056., 3200., 3200.])
        np.testing.assert_allclose(vehicle.channel_c[-4:], [169., 169., 128., 128.])
        for mode, direction in (('heave', [1., 1.]), ('roll', [1., -1.])):
            with self.subTest(mode=mode):
                mr_squared = full[f'{mode}_f_mr'] ** 2
                k = full[f'{mode}_f_k'] * mr_squared + 4056.
                c = full[f'{mode}_f_c'] * mr_squared + 169.
                np.testing.assert_allclose(lateral.ks @ direction, np.multiply(k, direction))
                np.testing.assert_allclose(lateral.cs @ direction, np.multiply(c, direction))
        np.testing.assert_allclose(lateral.kt, vehicle.kt[:2])
        np.testing.assert_allclose(lateral.ct, vehicle.ct[:2])
        np.testing.assert_allclose(lateral.static_load, vehicle.static_load[:2])
        np.testing.assert_allclose(lateral.masses, [vehicle.masses[0] * .6, vehicle.masses[1], 12., 12.])

    def test_linked_longitudinal_model_uses_selected_axle_on_both_ends(self):
        full = self.p['parameters']['full']
        full.update(tire_r_k=120000., mu_r=23., heave_r_mr=.8, corner_r_k=5000., corner_r_mr=1.2)
        settings = dict(duration=.32, start=.02, dt=.003)
        front = self.run_case('FR', parameter_axle='front', **settings)
        rear = self.run_case('FR', parameter_axle='rear', **settings)
        p = rear.project['parameters']['FR']
        self.assertEqual(rear.info['parameter_axle'], 'rear')
        self.assertEqual((p['lf'], p['lr'], p['Iy']), (full['lf'], full['lr'], full['Iyy'] / 2))
        self.assertEqual((p['kt'], p['mu']), (120000., 23.))
        for model in ('half_2dof', 'half'):
            with self.subTest(model=model):
                system = assemble('FR', model, p)
                np.testing.assert_allclose(system.ks.diagonal(), full['heave_r_k'] * .8**2 + 7200.)
                np.testing.assert_allclose(system.cs.diagonal(), full['heave_r_c'] * .8**2)
        self.assertGreater(np.max(np.abs(front.signals['Body F'] - rear.signals['Body F'])), .01)
        self.assertEqual(front.project['analysis']['FR']['parameter_axle'], 'front')

    def test_lr_combined_response_superposes_bump_and_lateral_inputs(self):
        options = dict(duration=.36, start=.04, dt=.002, target='left', ax=7., ay=-3.)
        bump = self.run_case('LR', 'half', scenario='bump', **options)
        lateral = self.run_case('LR', 'half', scenario='cornering', **options)
        combined = self.run_case('LR', 'half', scenario='bump_cornering', **options)
        np.testing.assert_array_equal(combined.signals['Road L'], bump.signals['Road L'])
        np.testing.assert_array_equal(combined.signals['ay'], lateral.signals['ay'])
        np.testing.assert_array_equal(combined.signals['ax'], 0.)
        np.testing.assert_allclose(combined.signals['Roll'], bump.signals['Roll'] + lateral.signals['Roll'], atol=1e-8)

    def test_timestep_convergence(self):
        self.p['parameters']['quarter']['ks']=1000.
        results=[self.run_case('quarter','single',scenario='step',start=0,duration=1.,dt=dt) for dt in (.01,.005,.0025)]
        p = results[0].project['parameters']['quarter']
        omega = np.sqrt((1000 * p['kt'] / (1000 + p['kt'])) / (p['ms'] + p['mu']))
        errors=[np.max(np.abs(r.signals['Body Q']-10*(1-np.cos(omega*r.time)))) for r in results]
        self.assertLess(errors[1],errors[0]/8)
        self.assertLess(errors[2],errors[1]/8)

    def test_rear_road_delay_uses_wheelbase_over_speed(self):
        p=parameters_for_analysis(self.p, 'FR');opt=self.p['analysis']['FR'];opt.update(target='all',start=.1,rear_delay=True)
        road=Road(assemble('FR','half',p),opt)
        delay=(p['lf']+p['lr'])/opt['speed']
        self.assertAlmostEqual(road.at(.2)[0][0],.01)
        self.assertEqual(road.at(.2)[0][1],0)
        self.assertAlmostEqual(road.at(.2+delay)[0][1],.01)

    def test_independent_right_road_velocity_is_used(self):
        p=parameters_for_analysis(self.p, 'LR');o=self.p['analysis']['LR'];o.update(scenario='sine',target='right')
        s=assemble('LR','half',p);road=Road(s,o);zr,vr,*_=road.at(.6)
        f=s.forces(.6,road)
        self.assertEqual(zr[0],0);self.assertEqual(vr[0],0)
        self.assertAlmostEqual(f[s.body_dof+1],p['kt']*zr[1]+p['ct']*vr[1])

    def test_front_rear_modal_and_direct_push_motion_ratios_are_independent(self):
        p=parameters_for_analysis(self.p, 'FR');p.update(mr_front=2.,mr_rear=3.,mr_corner=.5,k_front=10000.,k_rear=12000.,c_front=300.,c_rear=400.)
        s=assemble('FR','half',p)
        self.assertAlmostEqual(s.ks[0,0],p['k_heave_f']*4+10000*.25)
        self.assertAlmostEqual(s.ks[1,1],p['k_heave_r']*9+12000*.25)
        self.assertAlmostEqual(s.cs[0,0],p['c_heave_f']*4+300*.25)

    def test_random_seed_is_repeatable(self):
        a=self.run_case('quarter',scenario='random',duration=.3,start=.01,dt=.002)
        b=self.run_case('quarter',scenario='random',duration=.3,start=.01,dt=.002)
        np.testing.assert_array_equal(a.signals['Body Q'],b.signals['Body Q'])
        c=self.run_case('quarter',seed=1)
        self.assertGreater(np.max(np.abs(a.signals['Body Q']-c.signals['Body Q'])),.001)

    def test_endpoint_and_parameter_snapshot(self):
        r=self.run_case('quarter',duration=.371,dt=.013,start=.02)
        self.assertEqual(r.time[-1],.371);self.assertTrue(np.all(np.diff(r.time)>0))
        old=r.project['parameters']['quarter']['ks'];self.p['parameters']['quarter']['ks']=999
        self.assertEqual(r.project['parameters']['quarter']['ks'],old)

    def test_cancellation_and_invalid_parameters(self):
        with self.assertRaises(Cancelled):simulate(self.p,'full',cancel=lambda:True)
        for family,key,value in [('quarter','ms',0),('FR','ct',-1),('full','m',50),('LR','Ix',float('nan'))]:
            p=copy.deepcopy(self.p);p['parameters'][family][key]=value
            with self.assertRaises(ValueError):validate(p)
        p=copy.deepcopy(self.p);p['analysis']['LR'].update(model='2mass',scenario='accelerate')
        with self.assertRaises(ValueError):validate(p)


if __name__=='__main__':unittest.main()
