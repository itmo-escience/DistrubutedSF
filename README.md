##Запуск программы распределеннго мультиагентного моделирования#

###WIndows
Для запуска и дебага под windows требуется  Visual Studio 2010 с установленной библиотекой MPI. Для запуска и отладки требуется прописать соответствующие пути в свойствах проекта.

###Linux
Для запуска под линукс требуется скопировать исходные коды в отдельную папку, например _scratch/DistrSF/.

*   Source.cpp
*   AgentOnNodeInfo.h
*   AgentOnNodeInfo.cpp
*   makefile
*   *   SF
    *   *   include
        *   header files
    *   *   src
        *   source files

В ней должны находиться файлы Source.cpp AgentOnNodeInfo.h AgentOnNodeInfo.cpp а также мейкфаил.
также в этой папке должна находиться папка SF, а в ней include и src

для запуска программы на ломоносове требуется:
1. Получить доступ к нему и подключиться с помощью putty или winSCP
2. Скопировать исходники в папку scratch
3. Добавить необходимые пакеты
4. Собрать исходники с помощью мейкфайла командой "make dsf"
5. Подготовить скрипт для запуска программы с необходимыми параметрами
6. Поставить задачу в очередь с помощью команды sbatch <scriptfilename>

Выходные файлы появятся в папке _scratch

пример загрузки необходимых пакетов: 
```
module load intel/15.0.090
module load openmpi/1.8.4-icc
module load slurm
```

пример скрипта для запуска:
```

#!/bin/bash
#SBATCH -p regular4
#SBATCH -o <run_title>_%j.out
#SBATCH -e <run_title>_%j.err
#SBATCH -J DSF_<run_title>
#SBATCH —nodes=1
#SBATCH —time=01:00:00

ulimit -s unlimited
ulimit -l unlimited

time mpirun -n 8 -npernode 8 SF/dsf 0 0 300 10000 5 1000000 <run_title>

```
