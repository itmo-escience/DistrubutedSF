#!/bin/bash
#SBATCH --ntasks-per-node=8
#SBATCH --ntasks-per-core=1
#SBATCH -p regular4
#SBATCH -o %j.out
#SBATCH -e %j.err
#SBATCH -J DSF
#SBATCH -n 2
#SBATCH -N 2
#SBATCH --time=00:15:00
#SBATCH --mail-type=ALL
#SBATCH --mail-user=vad1611@yandex.ru

mpirun -n 2 SF/dsf 0 0 300 5000 5

