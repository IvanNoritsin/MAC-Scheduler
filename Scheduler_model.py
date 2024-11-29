import matplotlib.pyplot as plt
import numpy as np
import math

class Device:
    def __init__(self, id, data_to_send_kb, cqi):
        self.id = id
        self.data_to_send_kb = data_to_send_kb
        self.data_to_send = 0
        self.remaining_data = 0
        self.last_used_subcarrier = -1
        self.current_throughput = 0
        self.average_throughput = 1
        self.pf_metric = 0
        self.assigned_RBs = 0
        self.cqi = cqi
        
class MAC_Scheduler:
    def __init__(self, num_subcarriers, devices, time_slots):
        self.num_subcarriers = num_subcarriers
        self.devices = devices
        self.time_slots = time_slots
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
        
    def throughput_UE(self):
        for device in self.devices:
            throughput = self.mcs_table.get(device.cqi) * 7 * 12 * 2 * 1000 * self.num_subcarriers / 1048576
            device.current_throughput = throughput
                
            data_to_send_bits = device.data_to_send_kb * 8 * 1024
            bits_per_RB = (device.current_throughput * 1048576) / 2000                
            device.data_to_send = math.ceil(data_to_send_bits / bits_per_RB)
            device.remaining_data = device.data_to_send
    
    def round_robin(self):
        for device in self.devices:
            self.throughput_UE()
            
            total_throughput = 0
            active_devices = 0
        
        for t in range(self.time_slots):
            used_devices = set()
            used_subcarriers = set()
            
            if t % 2 == 0:
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
                used_devices.add(current_device)
                used_subcarriers.add(s)
                device.last_used_subcarrier = s
                self.device_index += 1
                active_devices += 1
                active_devices_per_slot += 1
                total_throughput += device.current_throughput
                total_throughput_per_slot += device.current_throughput
            
            if np.sum([device.remaining_data for device in self.devices]) == 0:
                break
            
            if t % 2 == 1:
                if active_devices_per_slot == 0:
                    self.avg_throughput_per_section_rr.append(0)
                else:
                    self.avg_throughput_per_section_rr.append(total_throughput_per_slot / active_devices_per_slot)     
            
            
        if active_devices == 0:
            self.avg_throughput_rr = 0
        else:
            self.avg_throughput_rr = total_throughput / active_devices   
        self.visualize_allocation(0)
        self.clear_allocation_matrix()

            
    def proportional_fair(self):
        for device in self.devices:
            self.throughput_UE()
            
            total_throughput = 0
            active_devices = 0
        
        for t in range(self.time_slots):
            sum_PF = 0
            
            if t % 2 == 0:
                print(f"\nВременной слот {t}:")
                
                total_throughput_per_slot = 0
                active_devices_per_slot = 0
                
                for device in self.devices:
                    if device.remaining_data > 0:
                        if t != 0:
                            device.average_throughput = 0.8 * device.average_throughput + 0.2 * device.current_throughput
                        device.pf_metric = device.current_throughput / device.average_throughput
                        sum_PF += device.pf_metric
                        
                check_assigned_RBs = 0
                for device in self.devices:
                    device.assigned_RBs = round((device.pf_metric / sum_PF) * (num_subcarriers * 2))
                    check_assigned_RBs += device.assigned_RBs
                    print(f"Устройство {device.id}: Current Throughput = {device.current_throughput:.3f}, "
                          f"Average Throughput = {device.average_throughput:.3f}, PF Metric = {device.pf_metric:.2f}, Assigned RBs = {device.assigned_RBs}") 
                    
                if check_assigned_RBs < num_subcarriers * 2:
                    unssigned_RBs = (num_subcarriers * 2) - check_assigned_RBs
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
  
            if t % 2 == 1:
                self.avg_throughput_per_section_pf.append(total_throughput_per_slot / active_devices_per_slot)
                
            if np.sum([device.remaining_data for device in self.devices]) == 0:
                break
            
        self.avg_throughput_pf = total_throughput / active_devices    
        self.visualize_allocation(1)
        self.clear_allocation_matrix()
        
    def best_cqi(self):
        for device in self.devices:
            self.throughput_UE()
            
            total_throughput = 0
            active_devices = 0
            
        for t in range(self.time_slots):
            sum_cqi = 0
                
            if t % 2 == 0:
                print(f"\nВременной слот {t}:")
                
                total_throughput_per_slot = 0
                active_devices_per_slot = 0
                
                for device in self.devices:
                    if device.remaining_data > 0:
                        sum_cqi += device.cqi

                check_assigned_RBs = 0
                for device in self.devices:
                    device.assigned_RBs = round((device.cqi / sum_cqi) * (num_subcarriers * 2))
                    check_assigned_RBs += device.assigned_RBs
                    print(f"Устройство {device.id}: CQI = {device.cqi}, Assigned RBs = {device.assigned_RBs}")            
                    
                
                if check_assigned_RBs < num_subcarriers * 2:
                    unssigned_RBs = (num_subcarriers * 2) - check_assigned_RBs
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

            if t % 2 == 1:
                self.avg_throughput_per_section_bcqi.append(total_throughput_per_slot / active_devices_per_slot)
                
            if np.sum([device.remaining_data for device in self.devices]) == 0:
                break
            
        self.avg_throughput_bcqi = total_throughput / active_devices
        self.visualize_allocation(2)
        self.clear_allocation_matrix()
        
        
    def clear_allocation_matrix(self):
        self.allocation_matrix = np.zeros((num_subcarriers, time_slots), dtype=int)
        self.usage_count = {device.id: 0 for device in self.devices}

         
    def visualize_allocation(self, scheduler_type):
        fig, ax = plt.subplots(figsize=(10, 6))
        if scheduler_type == 0:
            ax.set_title('Планирование ресурсов (Round Robin) для нисходящего канала')
        if scheduler_type == 1:
            ax.set_title('Планирование ресурсов (Proportional Fair) для нисходящего канала')
        if scheduler_type == 2:
            ax.set_title('Планирование ресурсов (Best CQI) для нисходящего канала')    
        ax.set_xlabel('Временные слоты')
        ax.set_ylabel('Ресурсные блоки')

        for (i, j), val in np.ndenumerate(self.allocation_matrix):
            ax.text(j, i, f'{val}', ha='center', va='center')

        cax = ax.matshow(self.allocation_matrix, cmap='tab20')
        fig.colorbar(cax, ax=ax, label='Номер устройства')

        ax.set_xticks(np.arange(self.time_slots))
        ax.set_yticks(np.arange(self.num_subcarriers))

        plt.show()
 
    def plot_average_throughput(self):
        x = np.arange(0, self.time_slots * 0.5 + 1, 1)
        plt.figure(figsize=(10, 6))
        if self.avg_throughput_per_section_rr:
            plt.plot(x, self.avg_throughput_per_section_rr, marker='o', linestyle='-', label="Round Robin")
        if self.avg_throughput_per_section_pf:
            plt.plot(x, self.avg_throughput_per_section_pf, marker='s', linestyle='-', label="Proportional Fair")
        if self.avg_throughput_per_section_bcqi:
            plt.plot(x, self.avg_throughput_per_section_bcqi, marker='p', linestyle='-', label="Best CQI")
        plt.title("Средняя пропускная способность")
        plt.xlabel("Время (мс)")
        plt.ylabel("Средняя пропускная способность (Мбит/с)")
        plt.legend()
        plt.grid()
        plt.xlim(0, self.time_slots * 0.5)
        plt.show()
        
    def plot_average_throughput_columns(self):
        labels = ['Round Robin', 'Proportional Fair', 'Best CQI']
        values = [self.avg_throughput_rr, self.avg_throughput_pf, self.avg_throughput_bcqi]
        
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
        
num_subcarriers = 15
time_slots = 40

devices = [
    Device(1, 250, 9),
    Device(2, 100, 9),
    Device(3, 170, 13),
    Device(4, 220, 1),
    Device(5, 70, 2),
    Device(6, 190, 12),
    Device(7, 100, 3),
    Device(8, 130, 12),
    Device(9, 160, 8),


]

scheduler = MAC_Scheduler(num_subcarriers, devices, time_slots)
scheduler.round_robin()
scheduler.proportional_fair()
scheduler.best_cqi()

scheduler.plot_average_throughput()
scheduler.plot_average_throughput_columns()