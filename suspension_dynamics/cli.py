"""Optional non-GUI entry point using exactly the desktop simulation engine."""
import argparse
from pathlib import Path

from .catalog import FAMILIES, MODELS
from .project import default_project, load_project
from .engine import simulate
from .storage import save_result, write_csv, write_report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project',type=Path)
    parser.add_argument('--family',choices=FAMILIES,default='full')
    parser.add_argument('--model')
    parser.add_argument('--scenario')
    parser.add_argument('--duration',type=float)
    parser.add_argument('--dt',type=float)
    parser.add_argument('--output',type=Path,required=True,help='Output prefix for NPZ, CSV and HTML')
    args=parser.parse_args()
    project=load_project(args.project) if args.project else default_project()
    for key in ('model','scenario','duration','dt'):
        value=getattr(args,key)
        if value is not None:project['analysis'][args.family][key]=value
    if project['analysis'][args.family]['model'] not in MODELS[args.family]:parser.error('Model does not belong to the selected family')
    result=simulate(project,args.family)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    save_result(str(args.output)+'.npz',result)
    write_csv(str(args.output)+'.csv',result)
    write_report(str(args.output)+'.html',result)
    print(f"{result.family}/{result.model}: {len(result.time)} samples, {result.info['internal_steps']} RK4 steps")


if __name__=='__main__':main()
