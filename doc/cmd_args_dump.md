## rosdiagramdump.py --help
```
usage: rosdiagramdump.py [-h] [-la] [--listtools]
                         {dumpclocdir,dumpclocpack,dumpcatkindeps,dumprosparam,dumprospack,dumprosmsg,dumprossrv,dumprosnode,dumprostopic,dumprosservice,dumproslaunch,dumpros,extractscripts}
                         ...

dump tools

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --listtools           List tools

subcommands:
  use one of tools

  {dumpclocdir,dumpclocpack,dumpcatkindeps,dumprosparam,dumprospack,dumprosmsg,dumprossrv,dumprosnode,dumprostopic,dumprosservice,dumproslaunch,dumpros,extractscripts}
                        one of tools
    dumpclocdir         dump result of 'cloc' command on given directory
    dumpclocpack        dump result of 'cloc' command on pack data
    dumpcatkindeps      dump catkin dependencies of packages in workspace
                        (from package.xml)
    dumprosparam        dump data from 'rosparam'
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
usage: rosdiagramdump.py dumpclocdir [-h] --clocrundir CLOCRUNDIR --outdir
                                     OUTDIR

dump result of 'cloc' command on given directory

optional arguments:
  -h, --help            show this help message and exit
  --clocrundir CLOCRUNDIR
                        Directory to analyze by 'cloc'
  --outdir OUTDIR       Output directory
```



## rosdiagramdump.py dumpclocpack --help
```
usage: rosdiagramdump.py dumpclocpack [-h] --packfile PACKFILE --outdir OUTDIR

dump result of 'cloc' command on pack data

optional arguments:
  -h, --help           show this help message and exit
  --packfile PACKFILE  List file dumped with `dumprospack` tool
  --outdir OUTDIR      Output directory
```



## rosdiagramdump.py dumpcatkindeps --help
```
usage: rosdiagramdump.py dumpcatkindeps [-h] --outdir OUTDIR

dump catkin dependencies of packages in workspace (from package.xml)

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py dumprosparam --help
```
usage: rosdiagramdump.py dumprosparam [-h] --outdir OUTDIR

dump data from 'rosparam'

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py dumprospack --help
```
usage: rosdiagramdump.py dumprospack [-h] --outdir OUTDIR

dump data from 'rospack'

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py dumprosmsg --help
```
usage: rosdiagramdump.py dumprosmsg [-h] --outdir OUTDIR

dump messages info

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py dumprossrv --help
```
usage: rosdiagramdump.py dumprossrv [-h] --outdir OUTDIR

dump services definitions

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py dumprosnode --help
```
usage: rosdiagramdump.py dumprosnode [-h] --outdir OUTDIR

dump nodes info

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py dumprostopic --help
```
usage: rosdiagramdump.py dumprostopic [-h] --outdir OUTDIR

dump topics info

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py dumprosservice --help
```
usage: rosdiagramdump.py dumprosservice [-h] --outdir OUTDIR

dump services info

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py dumproslaunch --help
```
usage: rosdiagramdump.py dumproslaunch [-h] --launchfile LAUNCHFILE --outdir
                                       OUTDIR

dump node names of launch file

optional arguments:
  -h, --help            show this help message and exit
  --launchfile LAUNCHFILE
                        Launch file
  --outdir OUTDIR       Output directory
```



## rosdiagramdump.py dumpros --help
```
usage: rosdiagramdump.py dumpros [-h] --outdir OUTDIR

dump majority of data

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## rosdiagramdump.py extractscripts --help
```
usage: rosdiagramdump.py extractscripts [-h] --outdir OUTDIR

extract embedded scripts to files

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```
