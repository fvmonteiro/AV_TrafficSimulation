import os

import numpy as np
import pandas as pd

from Vehicle import Vehicle
from vissim_interface import VissimInterface


class SSMDataWriter:
    """Helps saving aggregated SSM results to files"""
    file_extension = '.csv'

    def __init__(self, network_name):
        # TODO: this should be a static method from VissimInterface
        if network_name in VissimInterface.existing_networks:
            network_name = VissimInterface.existing_networks[network_name]
        elif network_name in VissimInterface.existing_networks.values():
            pass
        else:
            raise ValueError('Network "{}" is not in the list of valid '
                             'simulations\nCheck whether the network exists  '
                             'and add it to the VissimInterface attribute '
                             'existing_networks'.
                             format(network_name))
        self.file_base_name = network_name + '_SSM Results'
        self.network_data_dir = os.path.join(VissimInterface.networks_folder,
                                             network_name)

    def save_as_csv(self, data: pd.DataFrame,
                    autonomous_percentage: int = None):

        if autonomous_percentage is None:
            autonomous_percentage_folder = 'test'
        else:
            autonomous_percentage_folder = (str(autonomous_percentage)
                                            + '_percent_autonomous')
        max_sim_number = data['simulation_number'].iloc[-1]
        num_str = '_' + str(max_sim_number).rjust(3, '0')
        file_name = self.file_base_name + num_str + self.file_extension
        full_address = os.path.join(self.network_data_dir,
                                    autonomous_percentage_folder, file_name)
        data.to_csv(full_address, index=False)


class SyntheticDataWriter:
    """Creates synthetic data to help checking if the SSM computation is
    correct
    """

    file_extension = '.csv'
    data_dir = ('C:\\Users\\fvall\\Documents\\Research\\TrafficSimulation'
                '\\synthetic_data\\')

    def write_data(self, data):
        """

        :param data: pandas dataframe
        :return: None
        """
        data.to_csv(os.path.join(self.data_dir,
                                 'synthetic_data' + self.file_extension),
                    index=False)

    @staticmethod
    def create_data(vx=20, delta_v=0):
        """
        Creates simple data for tests.

        Only two vehicles, each one with constant speed. The kinematics do not
        have to be correct, i.e., we do not guarantee x = x0 + vt, since this is
        not important for the SSM tests.

        :param vx: Follower longitudinal velocity
        :param delta_v: Difference between leader and follower velocity.
         Note the order: vL - vF
        :return: pandas dataframe with the synthetic data
        """
        max_gap = 10 ** 2
        max_time = 10 ** 2
        gap_interval = 0.01
        vissim_max_delta_x = 250
        n_points = int(max_gap / gap_interval) + 1

        follower = dict()
        leader = dict()
        follower['time'] = np.round(np.linspace(0, max_time, n_points), 2)
        leader['time'] = follower['time']
        follower['veh_id'] = np.ones(n_points, dtype=int)
        leader['veh_id'] = follower['veh_id'] + 1
        follower['veh_type'] = Vehicle.NGSIM_CAR_ID * np.ones(n_points,
                                                              dtype=int)
        leader['veh_type'] = Vehicle.NGSIM_CAR_ID * np.ones(n_points,
                                                            dtype=int)
        follower['lane'] = np.ones(n_points, dtype=int)
        leader['lane'] = follower['lane']
        follower['x'] = np.zeros(n_points)
        leader['x'] = np.round(np.linspace(0, max_gap, n_points), 2)
        follower['vx'] = vx * np.ones(n_points)
        leader['vx'] = vx * np.ones(n_points) + delta_v
        follower['y'] = np.zeros(n_points)
        leader['y'] = np.zeros(n_points)
        follower['leader_id'] = leader['veh_id']
        leader['leader_id'] = leader['veh_id']  # if vehicle has no leader,
        # we set it as its own leader
        follower['delta_x'] = leader['x'] - follower['x']
        leader['delta_x'] = vissim_max_delta_x * np.ones(n_points)
        data = dict()
        for key in follower:
            data[key] = np.hstack((follower[key], leader[key]))

        return pd.DataFrame(data)
