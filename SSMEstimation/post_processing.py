import numpy as np
import pandas as pd
from scipy.stats import truncnorm

from Vehicle import Vehicle


class VehicleRecordPostProcessor:
    VISSIM = 'vissim'
    NGSIM = 'ngsim'
    SYNTHETIC = 'synthetic'
    integer_columns = {'veh_id': int, 'leader_id': int,
                       'veh_type': int, 'lane': int}

    def __init__(self, data_source, data):
        """
        :param data_source: string 'vissim' or 'ngsim'
        :param data: pandas dataframe with vehicle trajectory data
        """
        self.data_source = data_source.lower()
        self.veh_records = data

    def post_process_data(self):
        if self.data_source.lower() == self.VISSIM:
            self.post_process_vissim_data()
        elif self.data_source.lower() == self.NGSIM:
            self.post_process_ngsim_data()
        elif self.data_source == self.SYNTHETIC:
            self.post_process_synthetic_data()
        else:
            print('[{}] Trying to process data from unknown data source'.
                  format(self.__class__.__name__))
            return

    def post_process_vissim_data(self):
        """
        Process fzp vehicle record data file generated by VISSIM
        :return: None
        """
        veh_data = self.veh_records

        kph_to_mps = 1 / 3.6
        veh_data['vx'] = veh_data['vx'] * kph_to_mps

        # Only use samples after some vehicles have already left the simulation
        # n_discarded = 10  # number of vehicles
        # # Define warm up time as moment when first 10 vehicles have
        # # left simulation
        # discarded_idx = veh_data['veh_id'] <= n_discarded
        # if any(discarded_idx):
        #     warm_up_time = max(veh_data['time'].loc[discarded_idx])
        # else:
        #     warm_up_time = veh_data.iloc[0]['time']
        warm_up_time = 60
        self.remove_early_samples(warm_up_time)

        # By convention, if vehicle has no leader, we set it as its own leader
        veh_data['leader_id'].fillna(veh_data['veh_id'],
                                     inplace=True, downcast='infer')
        # Compute relative velocity to the vehicle's leader (own vel minus
        # leader vel) Note: we need this function because VISSIM output
        # 'SpeedDiff' is not always correct. It has been observed to equal
        # the vehicle's own speed at the previous time step.
        self.compute_values_relative_to_leader()

    def post_process_ngsim_data(self):
        """
        Process csv data file generated from NGSIM
        :return: None
        """
        veh_data = self.veh_records
        veh_data.astype(self.integer_columns, copy=False)

        columns_in_feet = ['x', 'y', 'vx', 'length', 'delta_x']
        foot_to_meter = 0.3048
        veh_data[columns_in_feet] *= foot_to_meter

        base_time = min(self.veh_records['time'])
        # From milliseconds to deciseconds
        veh_data['time'] = (veh_data['time'] - base_time) // 100
        veh_data.sort_values(by=['time', 'veh_id'], inplace=True)
        # Define warm up time as the first moment some vehicle has a leader
        warm_up_time = min(veh_data.loc[veh_data['leader_id'] > 0, 'time'])
        self.remove_early_samples(warm_up_time)
        # By convention, if vehicle has no leader, we set it as its own leader
        no_leader_idx = veh_data['leader_id'] == 0
        veh_data.loc[no_leader_idx, 'leader_id'] = veh_data['veh_id']

        veh_data['delta_x_old'] = veh_data['delta_x']  # for checks

        self.compute_values_relative_to_leader()

    def post_process_synthetic_data(self):
        """
        Process csv file synthetically generated
        :return: None
        """
        self.compute_values_relative_to_leader()

    def remove_early_samples(self, warm_up_time):
        """Remove samples with time below some warm up time
        :param warm_up_time: time below which samples are removed"""

        veh_data = self.veh_records
        below_warmup = veh_data.loc[veh_data['time'] <= warm_up_time].index
        print('Removing {} warm up time samples'.format(len(below_warmup)))
        veh_data.drop(index=below_warmup, inplace=True)

    def compute_delta_v_old(self):
        print('WARNING: deprecated method [June 2, 2021]')
        veh_data = self.veh_records
        print('Computing delta v for {} samples'.format(veh_data.shape[0]))

        veh_data.set_index('time', inplace=True)
        max_size = max(veh_data['veh_id']) + 1  # +1 due to zero indexing
        identity = np.eye(max_size)
        sim_times = np.unique(veh_data.index)
        percent = 0.1
        out_of_bounds_counter = 0
        for i in range(len(sim_times)):
            current_time = sim_times[i]
            current_data = veh_data.loc[[current_time]]
            adj_matrix = np.zeros([max_size, max_size])
            vel_vector = np.zeros(max_size)
            type_vector = np.zeros(max_size)

            veh_idx = current_data['veh_id'].values
            leader_idx = current_data['leader_id'].values
            # If leader is not in the current time, we proceed as if there
            # was no leader.
            out_of_bounds = leader_idx >= max_size
            out_of_bounds_counter += sum(out_of_bounds)
            leader_idx[out_of_bounds] = veh_idx[out_of_bounds]
            adj_matrix[veh_idx, leader_idx] = -1

            vel_vector[veh_idx] = current_data['vx']
            type_vector[veh_idx] = current_data['veh_type']
            delta_v = np.matmul((identity + adj_matrix), vel_vector)[veh_idx]
            np.fill_diagonal(adj_matrix, 0)
            leader_type = np.matmul(-adj_matrix, type_vector)[veh_idx]

            veh_data.loc[current_time, 'delta_v'] = delta_v
            veh_data.loc[current_time, 'leader_type'] = leader_type

            if i == int(percent * len(sim_times)):
                print('{}% done'.format(int(i / len(sim_times) * 100)))
                percent += 0.1

        print('Out of bounds leaders found in {} out of {} samples ({}%)'.
              format(out_of_bounds_counter, veh_data.shape[0],
                     int(out_of_bounds_counter / veh_data.shape[0] * 100)))

        veh_data.reset_index(inplace=True)

    def compute_values_relative_to_leader(self):
        """Computes bumper to bumper distance and relative speed to preceding
        vehicle, and adds the preceding vehicle type."""

        veh_data = self.veh_records
        total_samples = veh_data.shape[0]
        print('Adding distance, relative speed and leader type for {} '
              'samples'.format(total_samples))

        percent = 0.1
        out_of_bounds_idx = []
        grouped_by_time = veh_data.groupby('time')
        delta_v = np.zeros(total_samples)
        leader_type = np.zeros(total_samples)
        distance = np.zeros(total_samples)
        counter = 0
        for _, current_data in grouped_by_time:
            veh_idx = current_data['veh_id'].to_numpy()
            leader_idx = current_data['leader_id'].to_numpy()
            min_idx = min(veh_idx)
            max_idx = max(veh_idx)
            n_vehicles = max_idx - min_idx + 1
            vel_vector = np.zeros(n_vehicles)
            type_vector = np.zeros(n_vehicles)

            # If leader is not in the current time, we proceed as if there
            # was no leader (happens more often with NGSIM data)
            out_of_bounds_check = current_data['leader_id'] > max_idx
            if np.any(out_of_bounds_check):
                # Save the indices with issues to correct the original
                # dataframe after the loop
                out_of_bounds_idx.extend(list(out_of_bounds_check.
                                              index[out_of_bounds_check]))
                # Then correct the indices for this loop iteration
                leader_idx[out_of_bounds_check] = veh_idx[out_of_bounds_check]

            adjusted_idx = veh_idx - min_idx
            adjusted_leader_idx = leader_idx - min_idx

            vel_vector[adjusted_idx] = current_data['vx']
            delta_v[counter:counter + current_data.shape[0]] = (
                    vel_vector[adjusted_idx] - vel_vector[adjusted_leader_idx])

            type_vector[adjusted_idx] = current_data['veh_type']
            leader_type[counter:counter + current_data.shape[0]] = (
                type_vector[adjusted_leader_idx])

            distance[counter:counter + current_data.shape[0]] = (
                self.compute_distance_to_leader(current_data, adjusted_idx,
                                                adjusted_leader_idx))
            counter += current_data.shape[0]

            if counter >= percent * total_samples:
                print('{:.0f}%'.format(counter / total_samples * 100), end=',')
                percent += 0.1
        print()  # skip a line
        veh_data['delta_v'] = delta_v
        veh_data['leader_type'] = leader_type
        veh_data['delta_x'] = distance

        if len(out_of_bounds_idx) > 0:
            veh_data.loc[out_of_bounds_idx, 'leader_id'] = veh_data.loc[
                out_of_bounds_idx, 'veh_id']
            print('Found {} instances of leaders outside the simulation'.
                  format(len(out_of_bounds_idx)))

    def compute_distance_to_leader(self, veh_data, adjusted_idx,
                                   adjusted_leader_idx):
        """
        Computes the longitudinal distance between a vehicle's front
        bumper to the rear bumper of the leading vehicle
        :param veh_data: vehicle data during a single time step
        :param adjusted_idx: vehicle indices starting from zero
        :param adjusted_leader_idx: leader indices starting from zero
        """
        n = np.max(adjusted_idx) + 1
        if self.data_source == self.VISSIM:

            front_x_vector = np.zeros(n)
            front_y_vector = np.zeros(n)
            rear_x_vector = np.zeros(n)
            rear_y_vector = np.zeros(n)

            front_x_vector[adjusted_idx] = veh_data['front_x']
            rear_x_vector[adjusted_idx] = veh_data['rear_x']
            front_y_vector[adjusted_idx] = veh_data['front_y']
            rear_y_vector[adjusted_idx] = veh_data['rear_y']
            distance = np.sqrt((rear_x_vector[adjusted_leader_idx]
                                - front_x_vector[adjusted_idx]) ** 2
                               + (rear_y_vector[adjusted_leader_idx]
                                  - front_y_vector[adjusted_idx]) ** 2)

        elif self.data_source == self.NGSIM:
            length = np.zeros(n)
            length[adjusted_idx] = veh_data['length']
            leader_length = length[adjusted_leader_idx]
            distance = veh_data['delta_x'] - leader_length
        else:
            distance = veh_data['delta_x']

        return distance

    def create_time_bins_and_labels(self, period):
        """Creates equally spaced time intervals, generates labels
        that go with them and includes a time_interval column to the
        dataframe.

        :param period: time interval length
        :return: None; alters the data in place"""
        final_time = int(self.veh_records['time'].iloc[-1])
        interval_limits = []
        interval_labels = []
        for i in range(period, final_time + period,
                       period):
            interval_limits.append(i)
            interval_labels.append(str(i) + '-'
                                   + str(i + period))
        self.veh_records['time_interval'] = pd.cut(
            x=self.veh_records['time'], bins=interval_limits,
            labels=interval_labels[:-1])


class SSMEstimator:
    coded_ssms = {'TTC', 'DRAC', 'CPI', 'safe_gap', 'DTSG', 'vf_gap',
                  'exact_risk', 'estimated_risk'}
    ttc_threshold = 1.5  # [s]
    drac_threshold = 3.5  # [m/s2]

    def __init__(self, veh_data):
        self.veh_data = veh_data

    def include_ttc(self, safe_threshold: float = ttc_threshold):
        """
        Includes Time To Collision (TTC) and a flag indicating if the TTC is
        below a threshold to the the dataframe.
        TTC = deltaX/deltaV if follower is faster; otherwise infinity

        :param safe_threshold: [Optional] Threshold against which TTC values
         are compared.
        :return: None
        """
        veh_data = self.veh_data
        veh_data['TTC'] = float('nan')
        valid_ttc_idx = veh_data['delta_v'] > 0
        ttc = (veh_data['delta_x'].loc[valid_ttc_idx]
               / veh_data['delta_v'].loc[valid_ttc_idx])
        veh_data.loc[valid_ttc_idx, 'TTC'] = ttc
        veh_data['low_TTC'] = veh_data['TTC'] < safe_threshold

    def include_drac(self, safe_threshold: float = drac_threshold):
        """
        Includes Deceleration Rate to Avoid Collision (DRAC) and a flag
        indicating if the DRAC is above a threshold to the dataframe.
        DRAC = deltaV^2/(2.deltaX), if follower is faster; otherwise zero

        :param safe_threshold: [Optional] Threshold against which DRAC values
         are compared.
        :return: None
        """
        veh_data = self.veh_data
        veh_data['DRAC'] = 0
        valid_drac_idx = veh_data['delta_v'] > 0
        drac = (veh_data['delta_v'].loc[valid_drac_idx] ** 2
                / 2 / veh_data['delta_x'].loc[valid_drac_idx])
        veh_data.loc[valid_drac_idx, 'DRAC'] = drac
        veh_data['high_DRAC'] = veh_data['DRAC'] > safe_threshold

    def include_cpi(self, max_decel_data, is_default_vissim=True):
        """
        Includes Crash Probability Index (CPI) to the dataframe

        CPI = Prob(DRAC > MADR), where MADR is the maximum available
        deceleration rate. Formally, we should check the truncated Gaussian
        parameters for each velocity. However, the default VISSIM max
        decel is a linear function of the velocity and the other three
        parameters are constant. We make use of this to speed up this
        function.

        :param max_decel_data: dataframe with maximum deceleration per
        vehicle type per speed
        :param is_default_vissim: boolean to identify if data was generated
        using default VISSIM deceleration parameters
        :return: None
        """

        veh_types = np.unique(
            max_decel_data.index.get_level_values('veh_type'))
        df = self.veh_data
        if 'DRAC' not in df.columns:
            self.include_drac()
        df['CPI'] = 0
        # veh_types = np.unique(df['veh type'])
        for veh_type in veh_types:
            idx = (df['veh_type'] == veh_type) & (df['DRAC'] > 0)
            if is_default_vissim:
                first_row = max_decel_data.loc[veh_type, 0]
                possible_vel = max_decel_data.loc[veh_type].index
                min_vel = 0
                max_vel = max(possible_vel)
                decel_min_vel = max_decel_data.loc[veh_type, min_vel]['mean']
                decel_max_vel = max_decel_data.loc[veh_type, max_vel]['mean']
                madr_array = (decel_min_vel + (decel_max_vel - decel_min_vel)
                              / max_vel * df.loc[idx, 'vx'])
                df.loc[idx, 'CPI'] = truncnorm.cdf(df.loc[idx, 'DRAC'],
                                                   a=first_row['norm_min'],
                                                   b=first_row['norm_max'],
                                                   loc=(-1) * madr_array,
                                                   scale=first_row['std'])
            else:
                a_array = []
                b_array = []
                madr_array = []
                std_array = []
                for vel in df.loc[idx, 'vx']:
                    row = max_decel_data.loc[veh_type, round(vel, -1)]
                    a_array.append(row['norm_min'])
                    b_array.append(row['norm_max'])
                    madr_array.append(-1 * row['mean'])
                    std_array.append(row['std'])
                df.loc[idx, 'CPI'] = truncnorm.cdf(df.loc[idx, 'DRAC'],
                                                   a=a_array, b=b_array,
                                                   loc=madr_array,
                                                   scale=std_array)

    def include_collision_free_gap(self, same_type_gamma=1,
                                   consider_lane_change: bool = True):
        """
        Computes the collision free (safe) gap and adds it to the dataframe.
        If the vehicle violates the safe gap, the absolute value of the
        distance to the safe gap (DTSG) is also added to the dataframe. The
        DTSG column is padded with zeros.
        :param same_type_gamma: factor multiplying standard value of maximum
         braking of the leader when both leader and follower are of the same
         type. Values greater than 1 indicate more conservative assumptions
        :param consider_lane_change: if set to false, we don't consider the
         effects of lane change, i.e., we treat all situations as simple
         vehicle following. If set to true, we overestimate the risk by
         assuming a reduced max brake during lane changes.
        """

        ssm_1 = 'safe_gap'
        ssm_2 = 'DTSG'
        self.include_distance_based_ssm(
            ssm_1, same_type_gamma, consider_lane_change=consider_lane_change)

        if not consider_lane_change:
            ssm_1 += '_no_lane_change'
            ssm_2 += '_no_lane_change'
        self.veh_data[ssm_2] = self.veh_data['delta_x'] - self.veh_data[ssm_1]
        self.veh_data.loc[self.veh_data[ssm_2] > 0, ssm_2] = 0
        self.veh_data[ssm_2] = np.abs(self.veh_data[ssm_2])

    def include_vehicle_following_gap(self, same_type_gamma=1, rho=0.2,
                                      free_flow_velocity=None):
        """
        Includes time headway based desired vehicle following gap to the
        dataframe
        The vehicle following gap is an overestimation of the collision free
        gap which assumes:
        . (1-rho)vE(t) <= vL(t) <= vE(t), for all t
        . vE(t) <= Vf, for all t.

        :param same_type_gamma: factor multiplying standard value of maximum
         braking of the leader when both leader and follower are of the same
         type. Values greater than 1 indicate more conservative assumptions
        :param rho: defines the lower bound on the leader velocity following
         (1-rho)vE(t) <= vL(t). Must be in the interval [0, 1]
        :param free_flow_velocity: (optional) must be given in m/s
        :return:
        """
        self.include_distance_based_ssm('vf_gap', same_type_gamma, rho,
                                        free_flow_velocity)

    def include_exact_risk(self, same_type_gamma: float = 1,
                           consider_lane_change: bool = True):
        """
        Includes exact risk, computed as the relative velocity at
        collision time under the worst case scenario, to the dataframe

        :param consider_lane_change: if set to false, we don't consider the
         effects of lane change, i.e., we treat all situations as simple
         vehicle following. If set to true, we overestimate the risk by
         assuming a reduced max brake during lane changes.
        :param same_type_gamma: factor multiplying standard value of maximum
         braking of the leader when both leader and follower are of the same
         type. Values greater than 1 indicate more conservative assumptions
        """
        self.include_distance_based_ssm(
            'exact_risk', same_type_gamma,
            consider_lane_change=consider_lane_change)

    def include_estimated_risk(self, same_type_gamma=1, rho=0.2,
                               free_flow_velocity=None):
        """
        Includes estimated risk, which is an overestimation of the exact risk
        under certain assumptions, to the dataframe
        Assumptions:
        . (1-rho)vE(t) <= vL(t) <= vE(t), for all t
        . vE(t) <= Vf, for all t.

        :param same_type_gamma: factor multiplying standard value of maximum
         braking of the leader when both leader and follower are of the same
         type. Values greater than 1 indicate more conservative assumptions
        :param rho: defines the lower bound on the leader velocity following
         (1-rho)vE(t) <= vL(t). Must be in the interval [0, 1]
        :param free_flow_velocity: (optional) must be given in m/s
        :return:
        """
        self.include_distance_based_ssm('estimated_risk', same_type_gamma, rho,
                                        free_flow_velocity)

    def include_distance_based_ssm(self, ssm_name: str,
                                   same_type_gamma: float = 1,
                                   rho: float = 0.2,
                                   free_flow_velocity: float = None,
                                   consider_lane_change: bool = True):
        """
        Generic method to include one out of a set of distance based surrogate
        safety measures.
        :param ssm_name: {safe_gap, vf_gap,
         exact_risk, estimated_risk}
        :param same_type_gamma: factor multiplying standard value of maximum
         braking of the leader when both leader and follower are of the same
         type. Values greater than 1 indicate more conservative assumptions
        :param rho: defines the lower bound on the leader velocity following
         (1-rho)vE(t) <= vL(t). Must be in the interval [0, 1]
        :param consider_lane_change: if set to false, we don't consider the
         effects of lane change, i.e., we treat all situations as simple
         vehicle following. If set to true, we overestimate the risk by
         assuming a reduced max brake during lane changes.
        :param free_flow_velocity: (optional) must be given in m/s
        """
        veh_types = np.unique(self.veh_data['veh_type'])
        df = self.veh_data
        has_leader = df['veh_id'] != df['leader_id']
        if not consider_lane_change:
            ssm_name += "_no_lane_change"
        df[ssm_name] = 0

        for follower_type in veh_types:
            try:
                follower = Vehicle(follower_type, gamma=1)
            except KeyError:
                print('Follower of type {} not found. Skipping it.'.
                      format(follower_type))
                continue
            if not follower.is_relevant:
                print('Skipping follower of type ', follower.type)
                continue

            if free_flow_velocity is not None:
                follower.free_flow_velocity = free_flow_velocity
            follower_idx = (df['veh_type'] == follower_type) & has_leader

            for leader_type in veh_types:
                try:
                    if follower_type == leader_type:
                        gamma = same_type_gamma
                    else:
                        gamma = 1
                    leader = Vehicle(leader_type, gamma=gamma)
                except KeyError:
                    print('Leader of type {} not found. Skipping it.'.
                          format(leader_type))
                    continue
                if not leader.is_relevant:
                    print('Skipping leader of type ', leader.type)
                    continue

                veh_idx = follower_idx & (df['leader_type'] == leader_type)
                if ssm_name.startswith('safe_gap'):
                    df.loc[veh_idx, ssm_name] = (
                        self._compute_collision_free_gap(
                            veh_idx, follower, leader, consider_lane_change))
                elif ssm_name.startswith('vf_gap'):
                    df.loc[veh_idx, ssm_name] = (
                        self._compute_vehicle_following_gap(
                            veh_idx, follower, leader, rho))
                elif ssm_name.startswith('exact_risk'):
                    df.loc[veh_idx, ssm_name] = self._compute_exact_risk(
                        veh_idx, follower, leader, consider_lane_change)
                elif ssm_name.startswith('estimated_risk'):
                    df.loc[veh_idx, ssm_name] = (
                        self._compute_estimated_risk(veh_idx, follower,
                                                     leader, rho))
                else:
                    print('Unknown distance based SSM requested. Skipping...')
                    pass

    def _compute_collision_free_gap(self, veh_idx: pd.Series,
                                    follower: Vehicle, leader: Vehicle,
                                    consider_lane_change: bool = False):
        """
        The collision free is computed such that, under the worst case
        braking scenario, both vehicles achieve full stop without colliding

        :param veh_idx: boolean array indicating which vehicles of the
         dataset are being considered
        :param follower: following vehicle - object of the Vehicle class
        :param leader: leading vehicle - object of the Vehicle class
        :param consider_lane_change: if set to false, we don't consider the
         effects of lane change, i.e., we treat all situations as simple
         vehicle following. If set to true, we overestimate the risk by
         assuming a reduced max brake during lane changes.
        :return: safe gaps
        """
        follower_vel = self.veh_data.loc[veh_idx, 'vx'].values
        delta_vel = self.veh_data.loc[veh_idx, 'delta_v'].values
        leader_vel = follower_vel - delta_vel
        safe_gap = np.zeros(len(follower_vel))

        (follower_effective_max_brake,
         follower_effective_lambda1,
         _) = (
            self._get_braking_parameters_over_time(
                self.veh_data.loc[veh_idx, 'y'],
                follower,
                consider_lane_change))

        gamma = leader.max_brake / follower_effective_max_brake
        gamma_threshold = leader_vel / (follower_vel
                                        + follower_effective_lambda1)
        is_above = gamma >= gamma_threshold

        safe_gap[is_above] = (
            (follower_vel[is_above] ** 2 /
             (2 * follower_effective_max_brake[is_above])
             - leader_vel[is_above] ** 2 / (2 * leader.max_brake)
             + (follower_effective_lambda1[is_above] * follower_vel[is_above]
                / follower_effective_max_brake[is_above])
             + follower_effective_lambda1[is_above] ** 2 /
             (2 * follower_effective_max_brake[is_above])
             + follower.lambda0))
        if any(gamma) < 1:
            is_below = ~is_above
            brake_difference = (follower_effective_max_brake[is_below] -
                                leader.max_brake)
            safe_gap[is_below] = (
                (delta_vel[is_below] ** 2 / 2 / brake_difference
                 + (follower_effective_lambda1[is_below] * delta_vel[is_below]
                    / brake_difference)
                 + follower_effective_lambda1[is_below] ** 2 /
                 (2 * brake_difference)
                 + follower.lambda0))
        return safe_gap

    def _compute_vehicle_following_gap(self, veh_idx: pd.Series,
                                       follower: Vehicle, leader: Vehicle,
                                       rho: float = 0.2):
        """
        Computes time headway based vehicle following gap

        The vehicle following gap is an overestimation of the collision free
        gap which assumes:
        . (1-rho)vE(t) <= vL(t) <= vE(t), for all t
        . vE(t) <= Vf, for all t.

        :param veh_idx: boolean array indicating which vehicles of the
         dataset are being considered
        :param follower: following vehicle - object of the Vehicle class
        :param leader: leading vehicle - object of the Vehicle class
        :param rho: defines the lower bound on the leader velocity following
         (1-rho)vE(t) <= vL(t). Must be in the interval [0, 1]
        :return: vehicle following gaps
        """""
        h, d = follower.compute_vehicle_following_parameters(
            leader.max_brake, rho)
        return h * self.veh_data.loc[veh_idx, 'vx'] + d

    def _compute_exact_risk(self, veh_idx: pd.Series,
                            follower: Vehicle, leader: Vehicle,
                            consider_lane_change: bool = True):
        """
        Computes the exact risk, which is the relative velocity at collision
        time under the worst case braking scenario

        :param veh_idx: boolean array indicating which vehicles of the
         dataset are being considered
        :param follower: following vehicle - object of the Vehicle class
        :param leader: leading vehicle - object of the Vehicle class
        :param consider_lane_change: if set to false, we don't consider the
         effects of lane change, i.e., we treat all situations as simple
         vehicle following. If set to true, we overestimate the risk by
         assuming a reduced max brake during lane changes.
        :return: exact risks
        """
        gap = self.veh_data.loc[veh_idx, 'delta_x'].values
        follower_vel = self.veh_data.loc[veh_idx, 'vx'].values
        delta_vel = self.veh_data.loc[veh_idx, 'delta_v'].values
        leader_vel = follower_vel - delta_vel
        risk_squared = np.zeros(len(follower_vel))
        safe_gap = self.veh_data.loc[veh_idx, 'safe_gap'].values

        (follower_effective_max_brake,
         follower_effective_lambda1,
         follower_effective_tau_j) = (
            self._get_braking_parameters_over_time(
                self.veh_data.loc[veh_idx, 'y'],
                follower,
                consider_lane_change))

        gamma = leader.max_brake / follower_effective_max_brake
        gamma_threshold = leader_vel / (follower_vel +
                                        follower_effective_lambda1)
        is_gamma_above = gamma >= gamma_threshold

        # Gap thresholds
        # (note that delta_vel is follower_vel - leader_vel)
        gap_thresholds = [0] * 3
        gap_thresholds[0] = (
                follower.brake_delay
                * (follower.brake_delay / 2 * (follower.accel_t0
                                               + leader.max_brake)
                   + delta_vel))
        gap_thresholds[1] = (
                (follower.brake_delay + follower_effective_tau_j)
                * (follower_effective_lambda1 + delta_vel
                   - (follower.brake_delay + follower_effective_tau_j) / 2
                   * (follower_effective_max_brake - leader.max_brake))
                + follower.lambda0)
        gap_thresholds[2] = (
                leader_vel / leader.max_brake
                * (follower_effective_lambda1 + follower_vel
                   - (follower_effective_max_brake / leader.max_brake + 1)
                   * leader_vel / 2)
                + follower.lambda0)

        idx_case_1 = gap <= gap_thresholds[0]
        idx_case_2 = ((gap > gap_thresholds[1])
                      & (gap <= gap_thresholds[1]))
        idx_case_3 = ((gap > gap_thresholds[1])
                      & ((is_gamma_above
                          & (gap <= gap_thresholds[2]))
                         | (~is_gamma_above
                            & (gap <= safe_gap))))
        idx_case_4 = (is_gamma_above
                      & (gap > gap_thresholds[2])
                      & (gap <= safe_gap))

        risk_squared[idx_case_1] = (
                delta_vel[idx_case_1] ** 2
                + 2 * gap[idx_case_1] * (follower.accel_t0
                                         + leader.max_brake)
        )
        risk_squared[idx_case_2] = 0
        # In the code, delta_v = v_f - v_l (in the written documents it's
        # usually v_l - v_f)
        risk_squared[idx_case_3] = (
                (-delta_vel[idx_case_3]
                 - follower_effective_lambda1[idx_case_3]) ** 2
                - 2 * (follower_effective_max_brake[idx_case_3]
                       - leader.max_brake)
                * (gap[idx_case_3] - follower.lambda0)
        )
        risk_squared[idx_case_4] = (
                (follower_vel[idx_case_4]
                 + follower_effective_lambda1[idx_case_4]) ** 2
                - 2 * follower_effective_max_brake[idx_case_4] *
                (gap[idx_case_4] - follower.lambda0
                 + leader_vel[idx_case_4] ** 2 / 2 / leader.max_brake)
        )
        # Couple of sanity checks
        if any(idx_case_2):
            print('Collisions during jerk phase:', np.count_nonzero(idx_case_2))
            print('I guess it''s time to code severity for this case...')
        idx_issue = risk_squared < 0
        if np.any(idx_issue):
            print('{} negative risk samples'.format(np.count_nonzero(
                idx_issue)))
            risk_squared[idx_issue] = 0

        return np.sqrt(risk_squared)

    def _compute_estimated_risk(self, veh_idx: pd.Series,
                                follower: Vehicle, leader: Vehicle,
                                rho: float = 0.2):
        """
        Compute estimated risk, which is an overestimation of the exact risk
        under the following assumptions.
        . (1-rho)vE(t) <= vL(t) <= vE(t), for all t
        . vE(t) <= Vf, for all t.

        :param veh_idx: boolean array indicating which vehicles of the
         dataset are being considered
        :param follower: following vehicle - object of the Vehicle class
        :param leader: leading vehicle - object of the Vehicle class
        :param rho: defines the lower bound on the leader velocity following
         (1-rho)vE(t) <= vL(t). Must be in the interval [0, 1]
        :return: estimated risks
        """
        gamma = leader.max_brake / follower.max_brake
        gamma_threshold = ((1 - rho) * follower.free_flow_velocity
                           / (follower.free_flow_velocity + follower.lambda1))
        gap = self.veh_data.loc[veh_idx, 'delta_x'].values
        follower_vel = self.veh_data.loc[veh_idx, 'vx'].values

        if gamma >= gamma_threshold:
            estimated_risk_squared = (
                    ((1 - (1 - rho) ** 2 / gamma) * follower.free_flow_velocity
                     + 2 * follower.lambda1) * follower_vel
                    + follower.lambda1 ** 2
                    - 2 * follower.max_brake * (gap - follower.lambda0)
            )
        else:
            estimated_risk_squared = (
                    (rho ** 2 * follower.free_flow_velocity
                     + 2 * rho * follower.lambda1) * follower_vel
                    + follower.lambda1 ** 2
                    - 2 * follower.max_brake * (1 - gamma)
                    * (gap - follower.lambda0)
            )

        estimated_risk_squared[estimated_risk_squared < 0] = 0
        return np.sqrt(estimated_risk_squared)

    def _get_braking_parameters_over_time(self, lateral_position: pd.Series,
                                          vehicle: Vehicle,
                                          consider_lane_change: bool = True) \
            -> (np.array, np.array, np.array):
        """The vehicle maximum braking is reduced during lane change. This
        function determines when the vehicle is lane changing and returns
        arrays with values of maximum brake and lambda1 at each simulation
        step.

        :param lateral_position: relative position to the lane as provided by
         VISSIM, where 0.5 indicates the center of the lane.
        :param vehicle: object containing the vehicle's parameters
        :param consider_lane_change: if set to false, we don't consider the
         effects of lane change, i.e., we treat all situations as simple
         vehicle following. If set to true, we overestimate the risk by
         assuming a reduced max brake during lane changes.
        :return: Tuple of numpy arrays"""

        vehicle_effective_max_brake = (np.ones(len(lateral_position))
                                       * vehicle.max_brake)
        vehicle_effective_lambda1 = (np.ones(len(lateral_position))
                                     * vehicle.lambda1)
        vehicle_effective_tau_j = (np.ones(len(lateral_position))
                                   * vehicle.tau_j)
        if consider_lane_change:
            lane_change_idx = np.abs(lateral_position.values - 0.5) > 0.1
            vehicle_effective_max_brake[lane_change_idx] = (
                vehicle.max_brake_lane_change)
            vehicle_effective_lambda1[lane_change_idx] = (
                vehicle.lambda1_lane_change)
            vehicle_effective_tau_j[lane_change_idx] = (
                vehicle.tau_j_lane_change)

        return (vehicle_effective_max_brake,
                vehicle_effective_lambda1,
                vehicle_effective_tau_j)
