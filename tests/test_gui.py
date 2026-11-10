import copy
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
from tkinter import ttk

import numpy as np

from suspension_dynamics.catalog import MODELS, SCENARIOS, TARGETS
from suspension_dynamics.dashboard import FullVehicleDashboard
from suspension_dynamics.engine import simulate
from suspension_dynamics.gui import App, MODEL_NAMES
from suspension_dynamics.project import default_project
from suspension_dynamics.storage import statistics


class NativeWorkbenchTests(unittest.TestCase):
    def setUp(self):
        self.project=default_project()
        for opt in self.project['analysis'].values():opt.update(duration=.3,start=.03,dt=.003)
        self.app=App(self.project);self.app.update()

    def tearDown(self):
        self.app.dirty=False;self.app.close()

    def pump(self,predicate,timeout=20):
        deadline=time.monotonic()+timeout
        while not predicate():
            self.app.update()
            if time.monotonic()>deadline:self.fail('UI operation timed out')
            time.sleep(.003)
        self.app.update()

    def test_four_pages_and_result_switching_with_retained_animation_items(self):
        self.assertEqual(len(self.app.book.tabs()),4)
        expected_views = {'quarter': 'Side', 'LR': 'Front', 'FR': 'Side'}
        for family,panel in self.app.panels.items():
            with self.subTest(family=family):
                result=simulate(self.project,family)
                self.app.book.select(panel);panel.add_result(result);self.app.update()
                if family == 'full':
                    self.assertIsInstance(panel.motion, FullVehicleDashboard)
                    self.assertFalse(hasattr(panel, 'projection_control'))
                    canvas = panel.motion.canvas
                    initial_bar = canvas.coords('bar:FL')
                    self.assertTrue(initial_bar)
                    self.assertFalse(canvas.find_withtag('dashboard_cursor'))
                    readout, label, samples = panel.motion.readouts[0]
                else:
                    self.assertEqual(panel.motion.projection, expected_views[family])
                    self.assertEqual(panel.projection_control.winfo_class(), 'TLabel')
                    self.assertEqual(panel.projection_control['text'], f'{expected_views[family]} View')
                count=len(panel.motion.canvas.find_all());static=list(panel.chart.primitives)
                panel.seek(10);panel.seek(30);self.app.update()
                self.assertEqual(len(panel.motion.canvas.find_all()),count)
                if family == 'full':
                    self.assertNotEqual(canvas.coords('bar:FL'), initial_bar)
                    self.assertEqual(canvas.itemcget(readout, 'text'), f'{label} {samples[30]:.3f}')
                self.assertEqual(panel.chart.primitives,static)
                self.assertEqual(panel.current_index,30)
                self.assertEqual(panel.summary.get_children().__len__(),len(result.signals))
                self.assertTrue(panel.chart.canvas.find_withtag('playback'))
                panel.toggle_play();self.assertTrue(panel.playing)
                panel.pause();self.assertFalse(panel.playing)

    def test_minimum_window_keeps_chart_and_playback_visible(self):
        self.app.geometry('1160x760');self.app.update()
        for panel in self.app.panels.values():
            self.app.book.select(panel);self.app.update()
            motion = panel.motion.canvas
            chart = panel.chart.canvas
            self.assertTrue(motion.winfo_ismapped())
            self.assertLessEqual(motion.winfo_rootx() + motion.winfo_width(), chart.winfo_rootx())
            self.assertGreaterEqual(chart.winfo_width(), 300)
            self.assertTrue(panel.chart.canvas.winfo_ismapped())
            self.assertGreater(panel.chart.canvas.winfo_height(),120)
            if panel.family == 'full':
                panel.add_result(simulate(self.project, 'full'))
                self.app.update()
                self.assertTrue(motion.coords('bar:FL'))
                self.assertTrue(panel.motion.readouts)
            for widget in (panel.play_button,panel.scale,self.app.stop_button):
                self.assertTrue(widget.winfo_ismapped())
                self.assertLessEqual(widget.winfo_rooty()-self.app.winfo_rooty()+widget.winfo_height(),self.app.winfo_height())

    def test_parameters_apply_atomic_changes_and_preserve_old_snapshot(self):
        panel=self.app.panels['quarter'];panel.add_result(simulate(self.project,'quarter'))
        self.app.parameters();self.app.update();editor=self.app.editor
        original=self.app.project['parameters']['quarter']['ks']
        editor.entries['quarter','ks'].set('-1');editor.apply()
        self.assertEqual(self.app.project['parameters']['quarter']['ks'],original)
        self.assertTrue(editor.winfo_exists())
        editor.entries['quarter','ks'].set('32000');editor.apply();self.app.update()
        self.assertEqual(self.app.project['parameters']['quarter']['ks'],32000)
        self.assertEqual(panel.result.project['parameters']['quarter']['ks'],original)
        self.assertTrue(self.app.dirty)
        self.assertIn('快照',panel.result_status.get())

    def test_import_repairs_only_missing_fields_and_allows_cancel_or_retry(self):
        source = default_project()
        full = source['parameters']['full']
        del full['m']
        full.update(tf=None, tr='  ', lf=.8, body_twist_c=0, zeta=.65,
                    **source['parameters']['LR'])
        source.update(application='VehicleInitialSizingLab-DynamicsExport', parameters={'full': full})
        original = self.app.project
        base = Path(__file__).resolve().parents[1] / '.test-tmp'
        base.mkdir(exist_ok=True)

        def fill(dialog):
            fields = {field.location: (field.default, entry)
                      for field, entry in zip(dialog.fields, dialog.inputs, strict=True)}
            self.assertEqual({path: default for path, (default, _) in fields.items()},
                             {'parameters.full.m': 321., 'parameters.full.tf': 1.28,
                              'parameters.full.tr': 1.24})
            entry = fields['parameters.full.m'][1]
            entry.insert(0, '1')
            dialog.apply()
            self.assertTrue(dialog.winfo_exists())
            self.assertIn('總質量', dialog.error.cget('text'))
            self.assertIs(self.app.project, original)
            entry.delete(0, 'end')
            entry.insert(0, '340')
            dialog.apply()

        with tempfile.TemporaryDirectory(dir=base) as temp:
            path = Path(temp) / 'incomplete.json'
            contents = json.dumps(source)
            path.write_text(contents, encoding='utf-8')
            with (patch('suspension_dynamics.gui.filedialog.askopenfilename', return_value=str(path)),
                  patch('suspension_dynamics.gui.messagebox.showerror') as show_error):
                with patch.object(self.app, 'wait_window', side_effect=lambda dialog: dialog.destroy()):
                    self.app.open()
                self.assertIs(self.app.project, original)
                with patch.object(self.app, 'wait_window', side_effect=fill) as wait:
                    self.app.open()
                    wait.assert_called_once()
                show_error.assert_not_called()
            self.assertEqual(path.read_text(encoding='utf-8'), contents)

        repaired = self.app.project['parameters']['full']
        self.assertEqual([repaired[key] for key in ('m', 'tf', 'tr', 'lf', 'body_twist_c')],
                         [340., 1.28, 1.24, .8, 0])
        self.assertTrue(self.app.dirty)
        self.assertIsNone(self.app.project_path)

    def test_half_car_preview_and_analysis_follow_full_settings_and_axle_choice(self):
        panel = self.app.panels['FR']
        self.app.book.select(panel)
        self.assertEqual(panel.source_choice.get(), '前軸參數')
        panel.source_combo.current(1)
        panel.source_combo.event_generate('<<ComboboxSelected>>')
        self.app.parameters()
        self.app.update()
        editor = self.app.editor
        self.assertNotIn('FR', editor.auto_damping)
        wheel_fields = {key for family, key in editor.entries if family == 'full'
                        and key.startswith(('corner_', 'tire_'))}
        self.assertEqual(wheel_fields, {f'{component}_{axle}_{kind}'
                         for component in ('corner', 'tire') for axle in ('f', 'r') for kind in ('k', 'c', 'mr')})
        editor.entries['full', 'heave_r_k'].set('42000')
        editor.entries['full', 'tf'].set('1.4')
        editor.entries['full', 'tire_f_k'].set('90000')
        editor.entries['full', 'tire_r_k'].set('110000')
        editor.entries['full', 'corner_f_k'].set('2500')
        self.assertEqual(editor.entries['FR', 'k_heave_f'].get(), '84000')
        self.assertEqual(editor.entries['FR', 'k_heave_r'].get(), '84000')
        self.assertEqual(editor.entries['LR', 'track'].get(), '1.4')
        self.assertEqual(editor.entries['LR', 'kt'].get(), '90000')
        self.assertEqual(editor.entries['LR', 'k_left'].get(), '2500')
        self.assertNotIn(('LR', 'k_right'), editor.entries)
        self.assertEqual(editor.entries['FR', 'kt'].get(), '110000')
        editor.apply()
        panel.run()
        self.pump(lambda: not self.app.busy)
        result = panel.result
        self.assertEqual(result.info['parameter_axle'], 'rear')
        self.assertEqual(result.project['parameters']['FR']['k_heave_r'], 84000.)
        self.assertIn('後軸參數', panel.history.get())
        self.app.project['parameters']['full']['heave_r_k'] = 51000.
        panel.apply_result_inputs()
        self.assertEqual(self.app.project['parameters']['full']['heave_r_k'], 51000.)
        self.assertEqual(panel.source_choice.get(), '後軸參數')

    def test_quarter_linked_fields_are_readonly_and_auto_damping_tracks_the_corner(self):
        panel = self.app.panels['quarter']
        self.app.parameters()
        self.app.update()
        editor = self.app.editor
        quarter_page = editor.nametowidget(editor.notebook.select())
        fields = {str(widget.cget('textvariable')): widget for row in quarter_page.content.winfo_children()
                  for widget in row.winfo_children() if isinstance(widget, ttk.Entry)}
        for key in ('ms', 'mu', 'kt', 'ct'):
            self.assertTrue(fields[str(editor.entries['quarter', key])].instate(['readonly']))
        self.assertFalse(fields[str(editor.entries['quarter', 'ks'])].instate(['readonly']))
        editor.auto_damping['quarter'].set(True)
        previous_cs = float(editor.entries['quarter', 'cs'].get())
        editor.entries['full', 'mu_r'].set('20')
        editor.entries['full', 'lf'].set('1.2')
        self.assertNotAlmostEqual(float(editor.entries['quarter', 'cs'].get()), previous_cs)
        editor.apply()
        panel.source_combo.current(2)
        panel.source_combo.event_generate('<<ComboboxSelected>>')
        panel.run()
        self.pump(lambda: not self.app.busy)
        r = panel.result
        self.assertEqual(r.info['parameter_corner'], 'RL')
        p = r.project['parameters']['quarter']
        self.assertEqual(p['mu'], 20.)
        self.assertAlmostEqual(p['cs'], 2 * p['zeta'] * np.sqrt(p['ks'] * p['ms']) / p['mr'])
        self.assertNotAlmostEqual(p['cs'], self.app.project['parameters']['quarter']['cs'])

    def test_damping_checkbox_locks_live_estimate_and_remembers_mode(self):
        self.app.parameters()
        self.app.update()
        editor = self.app.editor
        field = editor.damping_fields['quarter', 'cs']
        # The selected notebook page exposes the actual checkbox to the user.
        page = editor.nametowidget(editor.notebook.select())
        checkbox = next(widget for row in page.content.winfo_children()
                        for widget in row.winfo_children() if isinstance(widget, ttk.Checkbutton))
        self.assertFalse(field.instate(['readonly']))
        checkbox.invoke()
        self.assertTrue(field.instate(['readonly']))
        style = ttk.Style(editor)
        self.assertNotEqual(style.lookup(field['style'], 'fieldbackground', ('readonly',)),
                            style.lookup(field['style'], 'fieldbackground'))
        before = field.get()
        field.insert(0, '123')
        self.assertEqual(field.get(), before)
        editor.entries['quarter', 'zeta'].set('0.6')
        expected = editor.values('quarter')['cs']
        self.assertAlmostEqual(float(field.get()), expected, places=6)
        self.assertNotEqual(field.get(), before)
        checkbox.invoke()
        self.assertFalse(field.instate(['readonly']))
        field.delete(0, 'end')
        field.insert(0, '1000')
        self.assertEqual(field.get(), '1000')
        checkbox.invoke()
        editor.apply()
        self.assertAlmostEqual(self.app.project['parameters']['quarter']['cs'], expected)
        self.app.parameters()
        self.app.update()
        self.assertTrue(self.app.editor.auto_damping['quarter'].get())
        self.assertTrue(self.app.editor.damping_fields['quarter', 'cs'].instate(['readonly']))
        self.assertNotIn('LR', self.app.editor.auto_damping)

    def test_run_all_models_keeps_ui_responsive_and_comparison_is_filtered(self):
        panel=self.app.panels['quarter'];heartbeats=[]
        def beat():
            heartbeats.append(self.app.busy)
            if self.app.busy:self.app.after(5,beat)
        self.app.calculate(panel,True);self.app.after(1,beat)
        self.pump(lambda:not self.app.busy)
        self.assertEqual(len(panel.records),4)
        self.assertTrue(any(heartbeats),'Tk events must run while simulation is active')
        panel.compare.set(True);panel.draw_chart()
        self.assertIn('4 個模型',panel.compare_info.get())
        previous=panel.result
        panel.vars['height'].set('.02');panel.run();self.pump(lambda:not self.app.busy)
        self.assertIn('1 個模型',panel.compare_info.get())
        self.assertNotEqual(previous.project['analysis']['quarter']['height'],panel.result.project['analysis']['quarter']['height'])

    def test_lr_lateral_acceleration_can_run_with_a_bump(self):
        panel = self.app.panels['LR']
        self.app.book.select(panel)
        panel.model.set(MODELS['LR']['2mass'])
        panel.model_changed()
        self.assertIn(SCENARIOS['cornering'], panel.scenario_combo.cget('values'))
        panel.scenario.set(SCENARIOS['bump_cornering'])
        panel.scenario_combo.event_generate('<<ComboboxSelected>>')
        self.app.update()
        for key in ('ay', 'ramp', 'height', 'period'):
            self.assertFalse(panel.input_widgets[key].instate(['disabled']))
        self.assertTrue(panel.input_widgets['ax'].instate(['disabled']))
        self.assertTrue(panel.target_combo.instate(['readonly']))
        panel.vars['ay'].set('-1.5')
        panel.run()
        self.pump(lambda: not self.app.busy)
        self.assertEqual(panel.result.signals['ay'][-1], -1.5)
        self.assertGreater(max(panel.result.signals['Road L']), 0)

    def test_quarter_iso_road_controls_and_animation_use_configured_speed(self):
        panel = self.app.panels['quarter']
        self.app.book.select(panel)
        choices = panel.scenario_combo.cget('values')
        for key in ('random', 'iso_a', 'iso_b', 'iso_c'):
            self.assertIn(SCENARIOS[key], choices)
        panel.scenario.set(SCENARIOS['iso_c'])
        panel.scenario_combo.event_generate('<<ComboboxSelected>>')
        self.app.update()
        for key in ('speed', 'seed'):
            self.assertFalse(panel.input_widgets[key].instate(['disabled']))
        for key in ('height', 'frequency', 'random_interval'):
            self.assertTrue(panel.input_widgets[key].instate(['disabled']))
        panel.vars['speed'].set('20')
        panel.vars['height'].set('0')
        panel.run()
        self.pump(lambda: not self.app.busy)
        result = panel.result
        self.assertEqual(result.info['iso_road']['speed_m_per_s'], 20.)
        self.assertTrue(all(row[2:5] == (None, None, None) for row in statistics(result)))
        panel.seek(45)
        motion = panel.motion
        wheel_x, datum_y = motion.to_canvas((0, 0, 0))
        road = np.array(motion.canvas.coords(motion.items[0]['ground'])).reshape(-1, 2)
        offsets = (road[:, 0] - wheel_x) / (20. * motion.pixels_per_metre)
        expected = np.interp(result.time[45] + offsets, result.time, result.signals['Road Q'])
        np.testing.assert_allclose(road[:, 1], datum_y - expected * motion.pixels_per_metre / 1000)

    def test_half_and_full_vehicle_iso_road_runs_on_selected_wheels(self):
        cases = (('LR', 'iso_a', 'right', 'Road R'), ('FR', 'iso_b', 'rear', 'Road R'),
                 ('full', 'iso_c', 'RR', 'Road RR'))
        for family, scenario, target, road_signal in cases:
            with self.subTest(family=family):
                panel = self.app.panels[family]
                self.app.book.select(panel)
                self.app.update()
                for key in ('iso_a', 'iso_b', 'iso_c'):
                    self.assertIn(SCENARIOS[key], panel.scenario_combo.cget('values'))
                panel.scenario.set(SCENARIOS[scenario])
                panel.scenario_combo.event_generate('<<ComboboxSelected>>')
                self.app.update()
                self.assertTrue(panel.target_combo.instate(['readonly']))
                for key in ('speed', 'seed'):
                    self.assertFalse(panel.input_widgets[key].instate(['disabled']))
                for key in ('height', 'frequency', 'random_interval'):
                    self.assertTrue(panel.input_widgets[key].instate(['disabled']))
                panel.target.set(TARGETS[target])
                panel.vars['speed'].set('20')
                panel.vars['seed'].set('32')
                panel.run()
                self.pump(lambda: not self.app.busy)
                result = panel.result
                self.assertEqual(result.info['iso_road']['class'], scenario[-1].upper())
                self.assertEqual(result.info['iso_road']['speed_m_per_s'], 20.)
                self.assertEqual(result.project['analysis'][family]['seed'], 32)
                self.assertGreater(np.max(np.abs(result.signals[road_signal])), 0.)
                for name, values in result.signals.items():
                    if name.startswith('Road ') and name != road_signal:
                        np.testing.assert_array_equal(values, 0.)

    def test_plot_svg_and_png_and_all_channels_are_exportable(self):
        panel=self.app.panels['full'];panel.add_result(simulate(self.project,'full'));self.app.book.select(panel);self.app.update()
        base=Path(__file__).resolve().parents[1]/'.test-tmp';base.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=base) as temp:
            for group in panel.result.groups:
                panel.group.set(group);panel.fill_signals();self.app.update()
                svg=panel.chart.svg();self.assertIn('<svg',svg);self.assertIn('</svg>',svg)
            try:import PIL
            except ImportError:return
            path=Path(temp)/'chart.png';panel.chart.png(path);self.assertGreater(path.stat().st_size,100)

    def test_async_cancel_leaves_controls_operable(self):
        panel=self.app.panels['full'];panel.vars['duration'].set('20')
        self.app.calculate(panel);self.app.after(10,self.app.cancel_event.set)
        self.pump(lambda:not self.app.busy)
        self.assertIn('停止',self.app.status.get())
        self.assertNotIn('disabled',panel.run_button.state())


if __name__=='__main__':unittest.main()
