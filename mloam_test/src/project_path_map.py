# https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/simple_optimize/simple_optimize.cpp

import numpy as np
import g2o 
import os

import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-n', '--max_iterations', type=int, default=10, help='perform n iterations')
parser.add_argument('-i', '--input', type=str, default='sphere2500.g2o', help='input file')
parser.add_argument('-o', '--output', type=str, default='', help='save resulting graph as file')
args = parser.parse_args()

def main():
    solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
    solver = g2o.OptimizationAlgorithmLevenberg(solver)

    optimizer = g2o.SparseOptimizer()
    optimizer.set_verbose(True)
    optimizer.set_algorithm(solver)

    optimizer.load(args.input)
    print('num vertices:{}'.format(len(optimizer.vertices())))
    print('num edges:{}'.format(len(optimizer.edges())))

    for i in range(len(optimizer.vertices())):
        vp_se3 = optimizer.vertex(i)
        print(dir(vp_se3))
        print(vp_se3.id)
        break


    # optimizer.initialize_optimization()
    # optimizer.optimize(args.max_iterations)

    # if len(args.output) > 0:
    #     optimizer.save(args.output)


if __name__ == '__main__':
    assert os.path.isfile(args.input), (
        'Please provide a existing .g2o file')
        
    main()