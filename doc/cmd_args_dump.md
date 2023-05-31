## rosdiagramdump.py --help
```
usage: rosdiagramdump.py [-h] [-la] [--listtools]
                         {dumpclocdir,dumpcatkindeps,dumprospack,dumprosmsg,dumprossrv,dumprosnode,dumprostopic,dumprosservice,dumproslaunch,dumpros,extractscripts}
                         ...

ROS diagram tools

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --listtools           List tools

subcommands:
  use one of tools

  {dumpclocdir,dumpcatkindeps,dumprospack,dumprosmsg,dumprossrv,dumprosnode,dumprostopic,dumprosservice,dumproslaunch,dumpros,extractscripts}
                        one of tools
    dumpclocdir         dump result of 'cloc' command on given directory
    dumpcatkindeps      dump catkin dependencies of packages in workspace
                        (from package.xml)
    dumprospack         dump data from 'rospack'
    dumprosmsg          dump messages info
    dumprossrv          dump services definitions
    dumprosnode         dump nodes info
    dumprostopic        dump topics info
    dumprosservice      dump services info
    dumproslaunch       dump node names of launch file
    dumpros             dump majority of data
    extractscripts      extract embedded scripts to files
```



## rosdiagramdump.py dumpclocdir --help
```
usage: rosdiagramdump.py dumpclocdir [-h] --clocrundir CLOCRUNDIR --outfile
                                     OUTFILE

optional arguments:
  -h, --help            show this help message and exit
  --clocrundir CLOCRUNDIR
                        Directory to analyze by 'cloc'
  --outfile OUTFILE     Output file
```



## rosdiagramdump.py dumpcatkindeps --help
```
usage: rosdiagramdump.py dumpcatkindeps [-h] --outdir OUTDIR

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py dumprospack --help
```
usage: rosdiagramdump.py dumprospack [-h] --outdir OUTDIR

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py dumprosmsg --help
```
usage: rosdiagramdump.py dumprosmsg [-h] --outdir OUTDIR

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py dumprossrv --help
```
usage: rosdiagramdump.py dumprossrv [-h] --outdir OUTDIR

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py dumprosnode --help
```
usage: rosdiagramdump.py dumprosnode [-h] --outdir OUTDIR

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py dumprostopic --help
```
usage: rosdiagramdump.py dumprostopic [-h] --outdir OUTDIR

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py dumprosservice --help
```
usage: rosdiagramdump.py dumprosservice [-h] --outdir OUTDIR

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py dumproslaunch --help
```
usage: rosdiagramdump.py dumproslaunch [-h] --launchfile LAUNCHFILE --outdir
                                       OUTDIR

optional arguments:
  -h, --help            show this help message and exit
  --launchfile LAUNCHFILE
                        Launch file
  --outdir OUTDIR       Output directory
```



## rosdiagramdump.py dumpros --help
```
usage: rosdiagramdump.py dumpros [-h] --outdir OUTDIR

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py extractscripts --help
```
usage: rosdiagramdump.py extractscripts [-h] --outdir OUTDIR

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```
