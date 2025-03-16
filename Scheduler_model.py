import matplotlib.pyplot as plt
import numpy as np
import math
import copy
import sys

class Device:
    def __init__(self, id, data_to_send_kb, coordinateX, coordinateY, speed, type_movement, angle):
        self.id = id
        self.data_to_send_kb = data_to_send_kb
        self.data_to_send = 0
        self.remaining_data = 0
        self.bits_per_RB = 0
        self.last_used_subcarrier = -1
        self.current_throughput = 0
        self.average_throughput = 1
        self.pf_metric = 0
        self.assigned_RBs = 0
        self.cqi = 0
        self.hms = 3
        self.coordinateX = coordinateX
        self.coordinateY = coordinateY
        self.coordinateX_array = [self.coordinateX]
        self.coordinateY_array = [self.coordinateY]
        self.speed = speed
        self.type_movement = type_movement
        self.angle = angle
        
    def movement_model(self, t):         
        if t % 2000 == 0:
            if self.type_movement == "walker":
                self.angle += np.random.uniform(-15, 15)
            elif self.type_movement == "car":
                self.angle += np.random.uniform(-7, 7)
            elif self.type_movement == "bicycle":
                self.angle += np.random.uniform(-15, 15)
            
        angle_rad = np.radians(self.angle)
        delta_x = (self.speed / 1000) * np.cos(angle_rad)
        delta_y = (self.speed / 1000) * np.sin(angle_rad)
        
        self.coordinateX += delta_x
        self.coordinateY += delta_y
        
        self.coordinateX_array.append(self.coordinateX)
        self.coordinateY_array.append(self.coordinateY)
            
    def convert_SNR_to_CQI(self, channelModel, d):
        SNR = channelModel.calculate_SNR(d)
        
        if SNR <= 2:
            self.cqi = 1
        elif SNR > 2 and SNR <= 4:
            self.cqi = 2
        elif SNR > 4 and SNR <= 6:
            self.cqi = 3
        elif SNR > 6 and SNR <= 8:
            self.cqi = 4
        elif SNR > 8 and SNR <= 10:
            self.cqi = 5
        elif SNR > 10 and SNR <= 12:
            self.cqi = 6
        elif SNR > 12 and SNR <= 14:
            self.cqi = 7     
        elif SNR > 14 and SNR <= 16:
            self.cqi = 8
        elif SNR > 16 and SNR <= 18:
            self.cqi = 9
        elif SNR > 18 and SNR <= 20:
            self.cqi = 10
        elif SNR > 20 and SNR <= 22:
            self.cqi = 11
        elif SNR > 22 and SNR <= 24:
            self.cqi = 12
        elif SNR > 24 and SNR <= 26:
            self.cqi = 13
        elif SNR > 26 and SNR <= 28:
            self.cqi = 14
        elif SNR > 28:
            self.cqi = 15
            
    def distance_UE_BS(self, x_BS, y_BS):
        distance = np.sqrt((self.coordinateX - x_BS)**2 + (self.coordinateY - y_BS)**2)
        return distance
            
        
class ChannelModel:
    def __init__(self, bandwidth, baseStation):
        self.bandwidth = bandwidth
        self.hms = 3
        self.baseStation = baseStation
    
    def COST231(self, d):
        A = 46.3
        B = 33.9
        a = 3.2 * (np.log10(11.75 * self.hms))**2 - 4.97
        s = np.where(d >= 1, 44.9 - 6.55 * np.log10(self.baseStation.carrierFreq), (47.88 + 13.9 * np.log10(self.baseStation.carrierFreq) - 13.9 * np.log10(self.baseStation.height)) * (1 / np.log10(50)))
        Lclutter = 3
        return A + B * np.log10(self.baseStation.carrierFreq) - 13.82 * np.log10(self.baseStation.height) - a + s * np.log10(d) + Lclutter
        
    def calculate_SNR(self, d):
        PathLoss = self.COST231(d)
        PowerSignal = self.baseStation.txPower - 2.9 + self.baseStation.antGain - PathLoss - 1 - 15
        PowerNoise = -174 + 10 * np.log10(self.bandwidth) + 6
        SNR = PowerSignal - PowerNoise
        return SNR
        
class BaseStation:
    def __init__(self, coordinateX, coordinateY):
        self.coordinateX = coordinateX
        self.coordinateY = coordinateY
        self.txPower = 46
        self.antGain = 21
        self.carrierFreq = 1800
        self.height = 100
        
class MAC_Scheduler:
    def __init__(self, num_subcarriers, devices, time_slots, channel_model):
        self.num_subcarriers = num_subcarriers
        self.devices = devices
        self.time_slots = time_slots
        self.channel_model = channel_model
        self.allocation_matrix = np.zeros((num_subcarriers, time_slots), dtype=int)
        self.device_index = 0
        self.avg_throughput_per_section_rr = [0]
        self.avg_throughput_per_section_pf = [0]
        self.avg_throughput_per_section_bcqi = [0]
        self.avg_throughput_rr = 0
        self.avg_throughput_pf = 0
        self.avg_throughput_bcqi = 0
        self.mcs_table = {
                            0: 0,
                            1: 0.1524,
                            2: 0.377,
                            3: 0.877,
                            4: 1.4764,
                            5: 1.914,
                            6: 2.4064,
                            7: 2.7306,
                            8: 3.3222,
                            9: 3.9024,
                            10: 4.5234,
                            11: 5.115,
                            12: 5.5544,
                            13: 6.2264,
                            14: 6.9072,
                            15: 7.4064
                        }
        
    def throughput_UE(self, device):
        throughput = self.mcs_table.get(device.cqi) * 7 * 12 * 2 * 1000 * self.num_subcarriers / 1048576
        device.current_throughput = throughput
            
        data_to_send_bits = device.data_to_send_kb * 8 * 1024
        device.bits_per_RB = (device.current_throughput * 1048576) / 2000                
        device.data_to_send = math.ceil(data_to_send_bits / device.bits_per_RB)
        device.remaining_data = device.data_to_send
            
    def RBs_to_DataKBs(self, device):
        data_to_send_bits = device.remaining_data * device.bits_per_RB
        device.data_to_send_kb = data_to_send_bits / 1024 / 8
        
    
    def round_robin(self):
        for device in self.devices:
            
            total_throughput = 0
            active_devices = 0
        
        for t in range(self.time_slots):
            if t == 0:
                for device in self.devices:
                    distance = device.distance_UE_BS(self.channel_model.baseStation.coordinateX, self.channel_model.baseStation.coordinateY)
                    distance = distance / 1000
                    device.convert_SNR_to_CQI(self.channel_model, distance)
                    self.throughput_UE(device)
                    
            if t % 2 == 0 and t != 0:          
                for device in self.devices:
                    self.RBs_to_DataKBs(device)
                    device.movement_model(t)
                    distance = device.distance_UE_BS(self.channel_model.baseStation.coordinateX, self.channel_model.baseStation.coordinateY)
                    distance = distance / 1000
                    device.convert_SNR_to_CQI(self.channel_model, distance)
                    self.throughput_UE(device)
            
            if t % 2 == 0:
                print(f"\nВременной слот {t}:")
                total_throughput_per_slot = 0
                active_devices_per_slot = 0
                  

            for s in range(self.num_subcarriers):
                available_devices = np.sum([device.remaining_data > 0 for device in self.devices])

                if available_devices == 0:
                    break

                tries = 0
                while (self.devices[self.device_index % len(self.devices)].remaining_data == 0):
                    self.device_index += 1
                    tries += 1

                    if tries >= len(self.devices):
                        break

                if tries >= len(self.devices):
                    break

                current_device = self.device_index % len(self.devices)
                device = self.devices[current_device]


                self.allocation_matrix[s, t] = current_device + 1
                device.remaining_data -= 1
                device.last_used_subcarrier = s
                self.device_index += 1
                active_devices += 1
                active_devices_per_slot += 1
                total_throughput += device.current_throughput
                total_throughput_per_slot += device.current_throughput
            
            if t % 2 == 1:
                if active_devices_per_slot == 0:
                    self.avg_throughput_per_section_rr.append(0)
                else:
                    self.avg_throughput_per_section_rr.append(total_throughput_per_slot / active_devices_per_slot)     
            
            self.devices = [device for device in self.devices if device.remaining_data > 0]
            
        if active_devices == 0:
            self.avg_throughput_rr = 0
        else:
            self.avg_throughput_rr = total_throughput / active_devices   
            
    def proportional_fair(self):
        for device in self.devices:
            
            total_throughput = 0
            active_devices = 0
        
        for t in range(self.time_slots):
            sum_PF = 0    
            
            if t == 0:
                for device in self.devices:
                    distance = device.distance_UE_BS(self.channel_model.baseStation.coordinateX, self.channel_model.baseStation.coordinateY)
                    distance = distance / 1000
                    device.convert_SNR_to_CQI(self.channel_model, distance)
                    self.throughput_UE(device)
                    
            if t % 2 == 0 and t != 0:          
                for device in self.devices:
                    self.RBs_to_DataKBs(device)
                    device.movement_model(t)
                    distance = device.distance_UE_BS(self.channel_model.baseStation.coordinateX, self.channel_model.baseStation.coordinateY)
                    distance = distance / 1000
                    device.convert_SNR_to_CQI(self.channel_model, distance)
                    self.throughput_UE(device)
            
            if t % 2 == 0:
                print(f"\nВременной слот {t}:")
                
                total_throughput_per_slot = 0
                active_devices_per_slot = 0
                
                for device in self.devices:
                    if device.remaining_data > 0:
                        if t != 0 and t % 10 == 0:
                            device.average_throughput = 0.8 * device.average_throughput + 0.2 * device.current_throughput
                        device.pf_metric = device.current_throughput / device.average_throughput
                        sum_PF += device.pf_metric
                        
                check_assigned_RBs = 0
                for device in self.devices:
                    device.assigned_RBs = round((device.pf_metric / sum_PF) * (self.num_subcarriers * 2))
                    check_assigned_RBs += device.assigned_RBs
                    print(f"Устройство {device.id}: Current Throughput = {device.current_throughput:.3f}, "
                          f"Average Throughput = {device.average_throughput:.3f}, PF Metric = {device.pf_metric:.2f}, Assigned RBs = {device.assigned_RBs}") 
                    
                if check_assigned_RBs < self.num_subcarriers * 2:
                    unssigned_RBs = (self.num_subcarriers * 2) - check_assigned_RBs
                    max_pf_metric = -1
                    for device in self.devices:
                        if device.pf_metric > max_pf_metric:
                            max_pf_metric = device.pf_metric
                            max_pf_device = device
                    max_pf_device.assigned_RBs += unssigned_RBs
 
            for s in range(self.num_subcarriers):
                max_pf_metric = -1
                selected_device = None
                
                for device in self.devices:
                    if device.remaining_data > 0 and device.assigned_RBs > 0:
                        if device.pf_metric > max_pf_metric:
                            max_pf_metric = device.pf_metric
                            selected_device = device
    
                if selected_device is None:
                    break
    
                self.allocation_matrix[s, t] = selected_device.id
                selected_device.remaining_data -= 1
                selected_device.assigned_RBs -= 1
                active_devices += 1
                active_devices_per_slot += 1
                total_throughput += selected_device.current_throughput
                total_throughput_per_slot += selected_device.current_throughput
                
            if t % 2 == 1 and self.devices:
                self.avg_throughput_per_section_pf.append(total_throughput_per_slot / active_devices_per_slot)
            elif t % 2 == 1 and not self.devices:
                self.avg_throughput_per_section_pf.append(0)
            
            self.devices = [device for device in self.devices if device.remaining_data > 0]
            
        self.avg_throughput_pf = total_throughput / active_devices    
        
    def best_cqi(self):
        for device in self.devices:
            
            total_throughput = 0
            active_devices = 0
            
        for t in range(self.time_slots):
            sum_cqi = 0
            
            if t == 0:
                for device in self.devices:
                    distance = device.distance_UE_BS(self.channel_model.baseStation.coordinateX, self.channel_model.baseStation.coordinateY)
                    distance = distance / 1000
                    device.convert_SNR_to_CQI(self.channel_model, distance)
                    self.throughput_UE(device) 
                    
            if t % 2 == 0 and t != 0:           
                for device in self.devices:
                    self.RBs_to_DataKBs(device)
                    device.movement_model(t)
                    distance = device.distance_UE_BS(self.channel_model.baseStation.coordinateX, self.channel_model.baseStation.coordinateY)
                    distance = distance / 1000
                    device.convert_SNR_to_CQI(self.channel_model, distance)
                    self.throughput_UE(device)
                
            if t % 2 == 0:
                print(f"\nВременной слот {t}:")
                
                total_throughput_per_slot = 0
                active_devices_per_slot = 0
                
                for device in self.devices:
                    if device.remaining_data > 0:
                        sum_cqi += device.cqi

                check_assigned_RBs = 0
                for device in self.devices:
                    device.assigned_RBs = round((device.cqi / sum_cqi) * (self.num_subcarriers * 2))
                    check_assigned_RBs += device.assigned_RBs
                    print(f"Устройство {device.id}: CQI = {device.cqi}, Assigned RBs = {device.assigned_RBs}")            
                    
                
                if check_assigned_RBs < self.num_subcarriers * 2 and self.devices:
                    unssigned_RBs = (self.num_subcarriers * 2) - check_assigned_RBs
                    max_cqi = -1
                    for device in self.devices:
                        if device.cqi > max_cqi:
                            max_cqi = device.cqi
                            max_cqi_device = device
                    max_cqi_device.assigned_RBs += unssigned_RBs
            
            for s in range(self.num_subcarriers):
                max_cqi = -1
                selected_device = None
                
                for device in self.devices:
                    if device.remaining_data > 0 and device.assigned_RBs > 0:
                        if device.cqi > max_cqi:
                            max_cqi = device.cqi
                            selected_device = device
    
                if selected_device is None:
                    break
     
                self.allocation_matrix[s, t] = selected_device.id
                selected_device.remaining_data -= 1
                selected_device.assigned_RBs -= 1
                active_devices += 1
                active_devices_per_slot += 1
                total_throughput += selected_device.current_throughput
                total_throughput_per_slot += selected_device.current_throughput
                
            if t % 2 == 1 and self.devices:
                self.avg_throughput_per_section_bcqi.append(total_throughput_per_slot / active_devices_per_slot)
            elif t % 2 == 1 and not self.devices:
                self.avg_throughput_per_section_bcqi.append(0)
                
            self.devices = [device for device in self.devices if device.remaining_data > 0]    
            
        self.avg_throughput_bcqi = total_throughput / active_devices
        
    def clear_allocation_matrix(self):
        self.allocation_matrix = np.zeros((self.num_subcarriers, self.time_slots), dtype=int)
        self.usage_count = {device.id: 0 for device in self.devices}

        
    def add_new_device(self, num_new_devices):
        
        for _ in range(num_new_devices):
            device_id = 1
            var = 0
            
            while var != 1: 
                if not self.devices:
                    var = 1
                    
                for device in self.devices:
                    if device.id == device_id:
                        device_id += 1
                    else:
                        var = 1
                        
            data_to_send_kb = np.random.randint(100, 1000)
            coordinateX = np.random.randint(-4000, 4000)
            coordinateY = np.random.randint(-4000, 4000)
            speed = np.random.randint(1, 25)
            type_movement = "linear"
            angle = np.random.randint(0, 360)
            
            new_device = Device(device_id, data_to_send_kb, coordinateX, coordinateY, speed, type_movement, angle)
            
            distance = new_device.distance_UE_BS(self.channel_model.baseStation.coordinateX, self.channel_model.baseStation.coordinateY)
            distance = distance / 1000
            new_device.convert_SNR_to_CQI(self.channel_model, distance)
            self.throughput_UE(new_device)
            
            self.devices.append(new_device)

def visualize_allocation(scheduler, scheduler_type, start_slot=0, end_slot=None):
    if end_slot is None or end_slot > scheduler.time_slots:
        end_slot = scheduler.time_slots
    
    allocation_matrix_cut = scheduler.allocation_matrix[:, start_slot:end_slot]

    fig, ax = plt.subplots(figsize=(10, 6))

    titles = {
        0: 'Планирование ресурсов (Round Robin) для нисходящего канала',
        1: 'Планирование ресурсов (Proportional Fair) для нисходящего канала',
        2: 'Планирование ресурсов (Best CQI) для нисходящего канала'
    }
    ax.set_title(titles.get(scheduler_type, "Неизвестный планировщик"))

    ax.set_xlabel('Временные слоты')
    ax.set_ylabel('Ресурсные блоки')

    for (i, j), val in np.ndenumerate(allocation_matrix_cut):
        ax.text(j, i, f'{val}', ha='center', va='center')

    cax = ax.matshow(allocation_matrix_cut, cmap='tab20')

    fig.colorbar(cax, ax=ax, label='Номер устройства')

    ax.set_xticks(np.arange(end_slot - start_slot))
    ax.set_xticklabels(np.arange(start_slot, end_slot))
    plt.setp(ax.get_xticklabels(), rotation=90)

    ax.set_yticks(np.arange(scheduler.num_subcarriers))

    plt.show()
    
def plot_average_throughput(avg_throughput_per_section_rr, avg_throughput_per_section_pf, avg_throughput_per_section_bcqi, time_slots):
    x = np.arange(0, time_slots * 0.5 + 1, 1)
    plt.figure(figsize=(10, 6))
    if avg_throughput_per_section_rr:
        plt.plot(x, avg_throughput_per_section_rr, linestyle='-', label="Round Robin")
    if avg_throughput_per_section_pf:
        plt.plot(x, avg_throughput_per_section_pf, linestyle='-', label="Proportional Fair")
    if avg_throughput_per_section_bcqi:
        plt.plot(x, avg_throughput_per_section_bcqi, linestyle='-', label="Best CQI")
    plt.title("Средняя пропускная способность")
    plt.xlabel("Время (мс)")
    plt.ylabel("Средняя пропускная способность (Мбит/с)")
    plt.legend()
    plt.grid()
    plt.xlim(0, time_slots * 0.5)
    plt.show()
    
def plot_average_throughput_columns(avg_throughput_rr, avg_throughput_pf, avg_throughput_bcqi):
    labels = ['Round Robin', 'Proportional Fair', 'Best CQI']
    values = [avg_throughput_rr, avg_throughput_pf, avg_throughput_bcqi]
    
    x = np.arange(len(labels))
    width = 0.25
    
    colors = ['blue', 'red', 'green']
    
    fig, ax = plt.subplots(figsize=(10, 6))
    bars = ax.bar(x, values, width, color=colors)
    
    for bar in bars:
        yval = bar.get_height()
        ax.text(bar.get_x() + bar.get_width() / 2, yval, round(yval, 2), ha='center', va='bottom')
    
    ax.set_ylabel('Средняя пропускная способность (Мбит/с)')
    ax.set_title('Сравнение средней пропускной способности')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    
    plt.show()
        
def plot_movement(devices, basestation):
    plt.figure(figsize=(10, 6))
    plt.plot(basestation.coordinateX, basestation.coordinateY, marker = 'o', label='Base Station')
    
    for device in devices:
        plt.plot(device.coordinateX_array, device.coordinateY_array, marker = 'o', markersize=1, label=f'Device {device.id}')
    
    plt.xlim(-5000, 5000)
    plt.ylim(-5000, 5000)
    plt.grid()
    plt.legend()

def get_num_RBs(bandwidth):
    num_RBs = 0
    if bandwidth == 1.4:
        num_RBs = 6
    elif bandwidth == 3:
        num_RBs = 15
    elif bandwidth == 5:
        num_RBs = 25
    elif bandwidth == 10:
        num_RBs = 50
    elif bandwidth == 15:
        num_RBs = 75
    elif bandwidth == 20:
        num_RBs = 100
    else:
        print("Невереная ширина канала!")
        sys.exit()
        
    return num_RBs
        
        
    
bandwidth = 3 # Ширина канала в МГц    
num_RBs = get_num_RBs(bandwidth)

simulation_time = 0.02 # Время симуляции в секундах
time_slots = int(simulation_time * 1000 * 2)

devices = [
    Device(1, 44000, 3800, -4000, 30, "train", 90),
    Device(2, 357000, 1700, 2000, 2, "walker", 225),
    Device(3, 220000, -3000, -2560, 16, "car", 270),
    Device(4, 417000, 1000, -1000, 5, "bicycle", 0),
    Device(5, 469000, -2300, 2500, 2, "walker", 45),
    Device(6, 310000, -1000, -300, 11, "car", 225),
    Device(7, 231000, 2500, 2800, 1, "walker", 270),
    Device(8, 75000, -4000, 4000, 6, "bicycle", 315),
    Device(9, 148000, 2300, -500, 13, "car", 180),
]

basestation = BaseStation(0, 0)
channel = ChannelModel(bandwidth * 1000000, basestation)
'''
devices_rr = copy.deepcopy(devices)
scheduler_rr = MAC_Scheduler(num_RBs, devices_rr, time_slots, channel)
scheduler_rr.round_robin()
visualize_allocation(scheduler_rr, 0, 0, 51)
visualize_allocation(scheduler_rr, 0, 100000, 100051)
plot_movement(devices_rr, basestation)
'''
'''
devices_pf = copy.deepcopy(devices)
scheduler_pf = MAC_Scheduler(num_RBs, devices_pf, time_slots, channel)
scheduler_pf.proportional_fair()
visualize_allocation(scheduler_pf, 1, 0, 51)
#visualize_allocation(scheduler_pf, 1, 100000, 100051)
plot_movement(devices_pf, basestation)
'''
devices_bcqi = copy.deepcopy(devices)
scheduler_bcqi = MAC_Scheduler(num_RBs, devices_bcqi, time_slots, channel)
scheduler_bcqi.best_cqi()
visualize_allocation(scheduler_bcqi, 2, 0, 51)
#visualize_allocation(scheduler_bcqi, 2, 100000, 100051)
plot_movement(devices_bcqi, basestation)
'''
plot_average_throughput_columns(scheduler_rr.avg_throughput_rr, 
                                scheduler_pf.avg_throughput_pf, 
                                scheduler_bcqi.avg_throughput_bcqi)
'''