class TimeSync:
    def __init__(self) -> None:

        self.msg_queue = []         # append message of specific sensor to queue
        self.last_pose_key = None
        self.debug_mode = False

    def add_to_queue(self, msg):
        self.msg_queue.append(msg)

    def debug_print(self, thing_to_print):
        if self.debug_mode:
            print(thing_to_print)
        else:
            pass

    def meas_is_before_node(self, meas_time, node_time):
        return meas_time < node_time
    
    def two_meas_on_either_side_node(self, meas_older_time, meas_newer_time, node_time):
        return meas_older_time < node_time and meas_newer_time > node_time
    
    def two_meas_both_after_node(self, meas_older_time, node_time):
        return meas_older_time > node_time
    
    def should_add_meas_to_node(self, meas_time, node_time, sensor_period):
        return abs(meas_time - node_time) <= sensor_period / 2 # So, a 1 Hz GPS needs to be +/- 0.5 seconds, or a 20 Hz sensor needs to be +/- 0.05 seconds
    
    def older_meas_is_closer_or_equal(self, old_meas_time, newer_meas_time, node_time):
        return abs(old_meas_time - node_time) <= abs(newer_meas_time - node_time)
    
    def time_match_unaries_to_nodes(self, nodes_without_unary, key_to_time, sensor_period):

        self.debug_print('\n\n')
        key_unary_pairs = [] # fill up with keys and the unary measurements that they match to
        i = 0
        keep_current_unary = False
        while (len(nodes_without_unary) > 0 or keep_current_unary) and len(self.msg_queue) >= 2:
            i+=1
            self.debug_print(f'Beginning iteration {i}')
            if keep_current_unary == False:    
                earliest_node_without = nodes_without_unary[0]
            else:
                self.debug_print('Still on the previously assigned earliest unary')
            keep_current_unary = False
            node_time = key_to_time[earliest_node_without]
            time_oldest_measurement = self.msg_queue[0][1]
            time_next_oldest_measurement = self.msg_queue[1][1]
            self.debug_print('===============================================')
            self.debug_print(f'Oldest Meas Time: {time_oldest_measurement}')
            self.debug_print(f'Newer Time: {time_next_oldest_measurement}')
            self.debug_print(f'Oldest Nodes without unary: {node_time}')
            self.debug_print('===============================================')
            self.debug_print('Checking ot see if the oldest measurement in the queue is before the node without a unary')
            if self.meas_is_before_node(time_oldest_measurement, node_time):
                self.debug_print('Oldest measurement is before the node without a unary, checking to see if the two oldest measurements straddle the node without a unary')
                if self.two_meas_on_either_side_node(time_oldest_measurement, time_next_oldest_measurement, node_time):
                    self.debug_print('They straddle, checking to see if we should add one of them')
                    if self.older_meas_is_closer_or_equal(time_oldest_measurement, time_next_oldest_measurement, node_time) and self.should_add_meas_to_node(time_oldest_measurement, node_time, sensor_period):
                        self.debug_print(f'MATCH: adding older sensor time {time_oldest_measurement} to node time {node_time}')
                        nodes_without_unary.pop(0)
                        key_unary_pairs.append((self.msg_queue.pop(0)[0], earliest_node_without, time_oldest_measurement))
                    elif self.should_add_meas_to_node(time_next_oldest_measurement, node_time, sensor_period):
                        self.msg_queue.pop(0)
                        self.debug_print(f'MATCH: adding newer sensor time {time_next_oldest_measurement} to node time {node_time}')
                        nodes_without_unary.pop(0)
                        key_unary_pairs.append((self.msg_queue.pop(0)[0], earliest_node_without, time_next_oldest_measurement))
                else:
                    self.debug_print('The two oldest measurements are before the node without a unary, so we get rid of the oldest since it is not the best measurement')
                    keep_current_unary = True
                    self.msg_queue.pop(0)
            elif self.two_meas_both_after_node(time_oldest_measurement, node_time):
                self.debug_print('Looks like it is actually after the node without a unary, checking to see if it is close enough to add')
                if self.should_add_meas_to_node(time_oldest_measurement, node_time, sensor_period):
                    self.debug_print('Looks like this unary should be added! We will pop the measurement an dmatch it to this node without a unary')
                    self.debug_print(f'MATCH: adding older sensor time {time_oldest_measurement} to node time {node_time}')
                    nodes_without_unary.pop(0)
                    key_unary_pairs.append((self.msg_queue.pop(0)[0], earliest_node_without, time_oldest_measurement))
                else:
                    self.debug_print('The unary is too far away. We will just forget about this node without a unary, and pop from the measurement queue')
                    nodes_without_unary.pop(0)
                    self.msg_queue.pop(0)

        
        self.debug_print('Out of the loop')
        self.debug_print(f'Number of nodes without unary: {len(nodes_without_unary)}')
        self.debug_print(f'keep_current_unary: {keep_current_unary}')
        self.debug_print(f'Length of queue: {len(self.msg_queue)}')

        print(f'Time pairs synced (sensor time stamp to node time):')
        for meas, key, time in key_unary_pairs:
            print(f'{time} : {key_to_time[key]}')
            

        return key_unary_pairs, nodes_without_unary
    