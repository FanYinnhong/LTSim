import sys, pdb
sys.path.append('..')

from libs.utils.world import World


def simulation():
    run_time = 300  # running time for simulation
    map_name = "Town02"
    world = World(run_time, map_name)
    world.load_map()  # init network
    world.save_map()
    demands_url = 'demands_2.txt'
    world.generate_flow(demands_url)  # init flow
    world.create_traffic_light()
    world.action()     # vehicle control
    world.visulizer(1)  # visulization, 1 for draw a new image


def main():
    simulation()


if __name__ == '__main__':
    main()
