import glob
import csv 
import os

class CFDataLoader:
    DT = 0.1
    def __init__(self, parent_path=''):
        self.full_state_paths = glob.glob(f'{parent_path}/**/full_state.csv', recursive=True)
        self.viconmarkers_paths = glob.glob(f'{parent_path}/**/viconmarkers.csv', recursive=True)
        self.SwarmCmds_paths = glob.glob(f'{parent_path}/**/SwarmCmds.csv', recursive=True)

        self.commands = []
        self.positions = []
        
        self.load_data()

    # def trim_data_paths(self):
    def load_data(self):
        for i in range(len(self.full_state_paths)):
            par_dir = os.path.dirname(self.full_state_paths[i])
            assert os.path.dirname(self.viconmarkers_paths[i]) == par_dir
            assert os.path.dirname(self.SwarmCmds_paths[i]) == par_dir

            full_state_data = []
            # print(self.full_state_paths[i])
            full_state_data_reader = csv.reader(open(self.full_state_paths[i], 'r'))
            for row in full_state_data_reader:
                full_state_data += [row]

            viconmarkers_data = []
            # print(self.viconmarkers_paths[i])
            viconmarkers_data_reader = csv.reader(open(self.viconmarkers_paths[i], 'r'))
            for row in viconmarkers_data_reader:
                viconmarkers_data += [row]
                
            SwarmCmds_data = []
            # print(self.SwarmCmds_paths[i])
            SwarmCmds_data_reader = csv.reader(open(self.SwarmCmds_paths[i], 'r'))
            for row in SwarmCmds_data_reader:
                SwarmCmds_data += [row]

            # idxs = self.get_synced_indices(full_state_data, viconmarkers_data, SwarmCmds_data)
            # print('\n'.join([str(c) for c in idxs]))
            # print(len(idxs), 'matching datapairs found.')

            dt = self.get_time_offset(full_state_data, viconmarkers_data, SwarmCmds_data)

            commands = self.get_commands(SwarmCmds_data, dt)
            # print('\n'.join([str(c) for c in commands]))
            # print(len(commands), 'commands sent')
            self.commands += [commands]

            positions = self.get_pos(viconmarkers_data, dt)
            # print('\n'.join([str(c) for c in positions]))
            # print(len(positions), 'position readings')
            self.positions += [positions]

    def get_time_offset(self, full_state_data, viconmarkers_data, SwarmCmds_data):
        t1 = int(full_state_data[1][4]) + int(full_state_data[1][4])*1e-9
        t2 = int(viconmarkers_data[1][4]) + int(viconmarkers_data[1][5])*1e-9
        t3 = int(SwarmCmds_data[1][6]) + int(SwarmCmds_data[1][7])*1e-9

        return t3

    def get_synced_indices(self, full_state_data, viconmarkers_data, SwarmCmds_data):
        idx1, idx2, idx3 = 1, 1, 1
        ret = []

        l = min(len(full_state_data), len(viconmarkers_data), len(SwarmCmds_data))-1

        while True: 
            if idx1 > l or idx2 > l or idx3 > l:
                break
            t1 = int(full_state_data[idx1][4]) + int(full_state_data[idx1][4])*1e-9
            t2 = int(viconmarkers_data[idx2][4]) + int(viconmarkers_data[idx2][5])*1e-9
            t3 = int(SwarmCmds_data[idx3][6]) + int(SwarmCmds_data[idx3][7])*1e-9

            mint = min(t1, t2, t3)
            maxt = max(t1, t2, t3)

            if maxt - mint < self.DT:
                ret += [[idx1, idx2, idx3]]
                idx1 += 1
                idx2 += 1
                idx3 += 1
            else:
                if t1 == mint:
                    idx1 += 1
                elif t2 == mint:
                    idx2 += 1
                elif t3 == mint:
                    idx3 += 1

        return ret

    def get_commands(self, SwarmCmds_data, dt):
        def tolist(l):
            return [float(item) for item in l.replace('[','').replace(']','').split(',')]
        
        cur_set = SwarmCmds_data[1][10]
        t = int(SwarmCmds_data[1][6]) + int(SwarmCmds_data[1][7])*1e-9 - dt
        ret = [[t, tolist(cur_set)]]
        for i in range(2, len(SwarmCmds_data)):
            if SwarmCmds_data[i][10] != cur_set:
                t = int(SwarmCmds_data[i][6]) + int(SwarmCmds_data[i][7])*1e-9 - dt
                ret += [[t, tolist(SwarmCmds_data[i][10])]]
                cur_set = SwarmCmds_data[i][10]
        return ret

    def get_pos(self, viconmarkers_data, dt):
        ret = []
        for i in range(1, len(viconmarkers_data)):
            t = int(viconmarkers_data[i][4]) + int(viconmarkers_data[i][5])*1e-9 - dt
            ret += [[t, [float(viconmarkers_data[i][13])/1000, float(viconmarkers_data[i][14])/1000, float(viconmarkers_data[i][15])/1000]]]
        return ret