## <a name="main_help"></a> rosdiagramdump.py --help
```
usage: rosdiagramdump.py [-h] [-la] [--listtools]
                         {dumpclocdir,dumpclocpack,dumpcatkindeps,dumprosparam,dumprospack,dumprosmsg,dumprossrv,dumprosnode,dumprostopic,dumprosservice,dumproslaunch,dumpros,dumprosrelative,extractscripts}
                         ...

dump tools

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --listtools           List tools

subcommands:
  use one of tools

  {dumpclocdir,dumpclocpack,dumpcatkindeps,dumprosparam,dumprospack,dumprosmsg,dumprossrv,dumprosnode,dumprostopic,dumprosservice,dumproslaunch,dumpros,dumprosrelative,extractscripts}
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
    dumprosrelative     dump majority of data (related msgs and srvs)
    extractscripts      extract embedded scripts to files
```



## <a name="dumpclocdir_help"></a> rosdiagramdump.py dumpclocdir --help
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



## <a name="dumpclocpack_help"></a> rosdiagramdump.py dumpclocpack --help
```
usage: rosdiagramdump.py dumpclocpack [-h] --packfile PACKFILE --outdir OUTDIR

dump result of 'cloc' command on pack data

optional arguments:
  -h, --help           show this help message and exit
  --packfile PACKFILE  List file dumped with `dumprospack` tool
  --outdir OUTDIR      Output directory
```



## <a name="dumpcatkindeps_help"></a> rosdiagramdump.py dumpcatkindeps --help
```
usage: rosdiagramdump.py dumpcatkindeps [-h] --outdir OUTDIR

dump catkin dependencies of packages in workspace (from package.xml)

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## <a name="dumprosparam_help"></a> rosdiagramdump.py dumprosparam --help
```
usage: rosdiagramdump.py dumprosparam [-h] --outdir OUTDIR

dump data from 'rosparam'

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## <a name="dumprospack_help"></a> rosdiagramdump.py dumprospack --help
```
usage: rosdiagramdump.py dumprospack [-h] --outdir OUTDIR

dump data from 'rospack'

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## <a name="dumprosmsg_help"></a> rosdiagramdump.py dumprosmsg --help
```
usage: rosdiagramdump.py dumprosmsg [-h] --outdir OUTDIR

dump messages info

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## <a name="dumprossrv_help"></a> rosdiagramdump.py dumprossrv --help
```
usage: rosdiagramdump.py dumprossrv [-h] --outdir OUTDIR

dump services definitions

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## <a name="dumprosnode_help"></a> rosdiagramdump.py dumprosnode --help
```
usage: rosdiagramdump.py dumprosnode [-h] --outdir OUTDIR

dump nodes info

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## <a name="dumprostopic_help"></a> rosdiagramdump.py dumprostopic --help
```
usage: rosdiagramdump.py dumprostopic [-h] --outdir OUTDIR

dump topics info

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## <a name="dumprosservice_help"></a> rosdiagramdump.py dumprosservice --help
```
usage: rosdiagramdump.py dumprosservice [-h] --outdir OUTDIR

dump services info

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## <a name="dumproslaunch_help"></a> rosdiagramdump.py dumproslaunch --help
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



## <a name="dumpros_help"></a> rosdiagramdump.py dumpros --help
```
usage: rosdiagramdump.py dumpros [-h] --outdir OUTDIR

dump majority of data

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## <a name="dumprosrelative_help"></a> rosdiagramdump.py dumprosrelative --help
```
usage: rosdiagramdump.py dumprosrelative [-h] --outdir OUTDIR

dump majority of data (related msgs and srvs)

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```



## <a name="extractscripts_help"></a> rosdiagramdump.py extractscripts --help
```
usage: rosdiagramdump.py extractscripts [-h] --outdir OUTDIR

extract embedded scripts to files

optional arguments:
  -h, --help       show this help message and exit
  --outdir OUTDIR  Output directory
```
