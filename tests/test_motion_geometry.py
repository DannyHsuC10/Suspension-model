"""Verify the visible tire profiles and metric aspect ratio on a real Canvas."""
import tkinter as tk
from types import SimpleNamespace
import unittest

import numpy as np

from suspension_dynamics.charts import MotionView, WHEEL_RADIUS, WHEEL_WIDTH, WHEEL_ITEMS
from suspension_dynamics.engine import Result
from suspension_dynamics.project import default_project, parameters_for_analysis


def recording(family):
    labels={'quarter':('Q',),'LR':('L','R'),'FR':('F','R'),'full':('FL','FR','RL','RR')}[family]
    signals={}
    for label in labels:
        for kind in ('Body','Wheel','Road'):
            signals[f'{kind} {label}']=np.array([0.,10. if kind=='Body' else 0.])
    project = default_project()
    project['parameters'][family] = parameters_for_analysis(project, family)
    return Result(family,'preview',np.array([0.,.1]),signals,{name:'mm' for name in signals},
                  {'位移':list(signals)},project,{'model_label':'Geometry verification'})


class MotionGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root=tk.Tk();cls.root.title('Suspension Dynamics Lab · Geometry verification')
        cls.view=MotionView(cls.root);cls.view.pack(fill='both',expand=True)
        cls.root.geometry('900x420');cls.root.update()

    @classmethod
    def tearDownClass(cls):cls.root.destroy()

    def setUp(self):
        self.view.gain=1.
        self.view.projection='Side'
        self.view.set_result(recording('full'))

    def points(self,name='wheel'):
        return np.array(self.view.canvas.coords(self.view.items[0][name])).reshape(-1,2)

    def test_side_tire_is_a_true_circle_for_every_family(self):
        for family in ('quarter','LR','FR','full'):
            with self.subTest(family=family):
                self.view.set_result(recording(family));points=self.points()
                width,height=np.ptp(points,axis=0)
                self.assertAlmostEqual(width,height,places=8)
                center=(points.min(axis=0)+points.max(axis=0))/2
                radius=self.view.pixels_per_metre*WHEEL_RADIUS
                np.testing.assert_allclose(np.linalg.norm(points-center,axis=1),radius,atol=1e-8)

    def test_front_tire_is_a_closed_rectangle_and_meets_road(self):
        self.view.projection='Front'
        self.view.set_result(recording('LR'))
        points=self.points()
        self.assertEqual(points.shape,(5,2));np.testing.assert_array_equal(points[0],points[-1])
        self.assertEqual(len(np.unique(points[:,0])),2);self.assertEqual(len(np.unique(points[:,1])),2)
        width,height=np.ptp(points,axis=0)
        self.assertAlmostEqual(width/height,WHEEL_WIDTH/(2*WHEEL_RADIUS),places=8)
        ground=np.array(self.view.canvas.coords(self.view.items[0]['ground'])).reshape(-1,2)
        self.assertAlmostEqual(points[:,1].max(),ground[:,1].mean(),places=8)

    def test_quarter_road_scrolls_and_aligns_current_input_with_wheel(self):
        result = recording('quarter')
        result.time = np.array([0., .1, .2, .3, .4])
        result.signals.update({'Body Q': np.zeros(5), 'Wheel Q': np.zeros(5),
                               'Road Q': np.array([0., 0., 20., 0., 0.])})
        self.view.set_result(result)
        items = self.view.canvas.find_all()
        before = self.points('ground')
        wheel_x, datum_y = self.view.to_canvas((0, 0, 0))
        self.assertGreater(before[np.argmin(before[:, 1]), 0], wheel_x)
        self.view.update_frame(2)
        at_bump = self.points('ground')
        self.assertAlmostEqual(at_bump[np.argmin(at_bump[:, 1]), 0], wheel_x)
        self.assertAlmostEqual(at_bump[:, 1].min(), datum_y - .020 * self.view.pixels_per_metre)
        self.view.update_frame(4)
        after = self.points('ground')
        self.assertLess(after[np.argmin(after[:, 1]), 0], wheel_x)
        self.assertEqual(self.view.canvas.itemcget(self.view.road_fill, 'state'), 'normal')
        self.assertEqual(self.view.canvas.find_all(), items)

    def test_metric_scale_survives_resize_pan_zoom_and_animation(self):
        for projection in ('Side','Front'):
            self.view.projection=projection
            for size in ('1000x300','460x650','900x420'):
                self.root.geometry(size);self.root.update();self.view.update_frame(0)
                self.view.zoom(SimpleNamespace(delta=120))
                self.view.drag=(0,0);self.view.pan(SimpleNamespace(x=17,y=23))
                scale=self.view.pixels_per_metre
                self.view.update_frame(1)
                self.assertEqual(self.view.pixels_per_metre,scale)
                origin=np.array(self.view.to_canvas((0,0,0)))
                horizontal=(1,0,0) if projection=='Side' else (0,1,0)
                dx=np.linalg.norm(np.array(self.view.to_canvas(horizontal))-origin)
                dy=np.linalg.norm(np.array(self.view.to_canvas((0,0,1)))-origin)
                self.assertAlmostEqual(dx,dy,places=8)

    def test_view_switch_uses_one_cylinder_and_retains_canvas_items(self):
        count=len(self.view.canvas.find_all())
        for projection in ('Iso','Front','Side','Iso'):
            self.view.projection=projection;self.view.update_frame(0)
            self.assertEqual(len(self.view.canvas.find_all()),count)
            visible=[name for name in WHEEL_ITEMS if self.view.canvas.itemcget(self.view.items[0][name],'state')!='hidden']
            self.assertEqual(len(visible),6 if projection=='Iso' else 1)


if __name__=='__main__':unittest.main()
