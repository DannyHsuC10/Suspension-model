import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


class StandaloneTests(unittest.TestCase):
    def test_isolated_copy_runs_every_family_without_sibling_projects(self):
        root=Path(__file__).resolve().parents[1]
        base=root/'.test-tmp';base.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=base) as temp:
            target=Path(temp)/'standalone'
            shutil.copytree(root/'suspension_dynamics',target/'suspension_dynamics',ignore=shutil.ignore_patterns('__pycache__'))
            code="""
import sys
sys.path.insert(0,sys.argv[1])
from suspension_dynamics.project import default_project
from suspension_dynamics.catalog import MODELS
from suspension_dynamics.engine import simulate
from suspension_dynamics.gui import App
p=default_project()
for family,models in MODELS.items():
 for model in models:
  p['analysis'][family].update(model=model,duration=.05,start=.01,dt=.005)
  assert simulate(p,family).time[-1]==.05
assert 'matplotlib' not in sys.modules
assert not {'vehicle_lab', 'suspension_geometry_lab', 'suspension_lab'}.intersection(sys.modules)
app=App();app.update();app.dirty=False;app.close()
print('All 13 models and native UI run independently')
"""
            proc=subprocess.run([sys.executable,'-I','-c',code,str(target)],cwd=temp,capture_output=True,text=True,timeout=30)
            self.assertEqual(proc.returncode,0,proc.stdout+proc.stderr)
            self.assertIn('All 13',proc.stdout)


if __name__=='__main__':unittest.main()
