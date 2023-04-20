import xml.etree.ElementTree as ET
import tkinter as tk
from tkinter import ttk

    
# Function to update the XML values
def update_xml():
    new_nt_val = nt_slider.get()  # Get value from commons slider
    new_speed_val = speed_slider.get()  # Get value from commons slider
    new_epssdf_val = eps_sdf_slider.get()  # Get value from commons slider
    new_sig0_val = sig0_slider.get()  # Get value from commons slider
    new_sigT_val = sigT_slider.get()  # Get value from commons slider
    new_eta_val = eta_slider.get()  # Get value from commons slider
    new_stop_err_val = stop_err_slider.get()  # Get value from commons slider
    
    new_cost_sigma1_val = exp1_collision_slider.get()  # Get value from experiment1 slider
    new_cost_sigma2_val = exp2_collision_slider.get()  # Get value from experiment1 slider
    new_cost_sigma3_val = exp3_collision_slider.get()  # Get value from experiment1 slider
    new_cost_sigma4_val = exp4_collision_slider.get()  # Get value from experiment1 slider

    # Update Commons values
    root.find('Commons/nt').text = str(new_nt_val)
    root.find('Commons/eps_sdf').text = str(new_epssdf_val)
    root.find('Commons/speed').text = str(new_speed_val)
    root.find('Commons/sig0').text = str(new_sig0_val)
    root.find('Commons/sigT').text = str(new_sigT_val)
    root.find('Commons/eta').text = str(new_eta_val)
    root.find('Commons/stop_err').text = str(new_stop_err_val)

    # Update Experiment1/start_pos/x value
    root.find('Experiment1/cost_sigma').text = str(new_cost_sigma1_val)
    root.find('Experiment2/cost_sigma').text = str(new_cost_sigma2_val)
    root.find('Experiment3/cost_sigma').text = str(new_cost_sigma3_val)
    root.find('Experiment4/cost_sigma').text = str(new_cost_sigma4_val)

    # Save updated XML to a file
    tree.write('/home/hongzhe/git/VIMP/vimp/helpers/example.xml')
    print('XML values updated and saved.')

# Load the XML file
tree = ET.parse('/home/hongzhe/git/VIMP/vimp/helpers/example.xml')
root = tree.getroot()

# Create tkinter window
window = tk.Tk()
window.title("XML Value Updater")

# ------------------ Create a label widget to display the value -----------------
label_nt = tk.Label(window, text="nt: 0")
# label_nt.pack()
label_nt.grid(row=0, column=0, columnspan=2)  # Place nt_slider in the first row, first column, spanning 2 columns

def update_label_nt(value):
    label_nt.config(text="nt: {}".format(value))
    
# Create sliders for Commons nt values
nt_slider = ttk.Scale(window, from_=0, to=100, orient=tk.HORIZONTAL, length=300, command=update_label_nt)
nt_slider.set(float(root.find('Commons/nt').text))
nt_slider.grid(row=1, column=0, columnspan=2)  # Place nt_slider in the first row, first column, spanning 2 columns


# ------------------ Create a label widget to display the value -----------------
label_epssdf = tk.Label(window, text="eps_sdf: 0")
# label_nt.pack()
label_epssdf.grid(row=2, column=0, columnspan=2)  # Place nt_slider in the first row, first column, spanning 2 columns

def update_label_epssdf(value):
    label_epssdf.config(text="eps_sdf: {}".format(value))
    
# Create sliders for Commons eps_sdf values
eps_sdf_slider = ttk.Scale(window, from_=0, to=3, orient=tk.HORIZONTAL, length=300, command=update_label_epssdf)
eps_sdf_slider.set(float(root.find('Commons/eps_sdf').text))
eps_sdf_slider.grid(row=3, column=0, columnspan=2)  # Place nt_slider in the first row, first column, spanning 2 columns


# ------------------ Create a label widget to display the value -----------------
label_speed = tk.Label(window, text="speed: 0")
# label_nt.pack()
label_speed.grid(row=4, column=0, columnspan=2)  # Place nt_slider in the first row, first column, spanning 2 columns

def update_label_speed(value):
    label_speed.config(text="speed: {}".format(value))
    
# Create sliders for Commons speed values
speed_slider = ttk.Scale(window, from_=0, to=2, orient=tk.HORIZONTAL, length=300, command=update_label_speed)
speed_slider.set(float(root.find('Commons/speed').text))
speed_slider.grid(row=5, column=0, columnspan=2)  # Place nt_slider in the first row, first column, spanning 2 columns

# ------------------ Create a label widget to display the value -----------------
label_sig0 = tk.Label(window, text="sig0: 0")
# label_nt.pack()
label_sig0.grid(row=6, column=0, columnspan=2)  # Place nt_slider in the first row, first column, spanning 2 columns

def update_label_sig0(value):
    label_sig0.config(text="sig0: {}".format(value))
    
# Create sliders for Commons sig0 values
sig0_slider = ttk.Scale(window, from_=0, to=1, orient=tk.HORIZONTAL, length=300, command=update_label_sig0)
sig0_slider.set(float(root.find('Commons/sig0').text))
sig0_slider.grid(row=7, column=0, columnspan=2)  # Place nt_slider in the first row, first column, spanning 2 columns


# ------------------ Create a label widget to display the value -----------------
label_sigT = tk.Label(window, text="sigT: 0")
# label_nt.pack()
label_sigT.grid(row=8, column=0, columnspan=2)  # Place nt_slider in the first row, first column, spanning 2 columns

def update_label_sigT(value):
    label_sigT.config(text="sigT: {}".format(value))
    
# Create sliders for Commons sigT values
sigT_slider = ttk.Scale(window, from_=0, to=1, orient=tk.HORIZONTAL, length=300, command=update_label_sigT)
sigT_slider.set(float(root.find('Commons/sigT').text))
sigT_slider.grid(row=9, column=0, columnspan=2)  # Place nt_slider in the first row, first column, spanning 2 columns


# ------------------ Create a label widget to display the value -----------------
label_eta = tk.Label(window, text="eta: 0")
# label_nt.pack()
label_eta.grid(row=10, column=0, columnspan=2)  # Place nt_slider in the first row, first column, spanning 2 columns

def update_label_eta(value):
    label_eta.config(text="eta: {}".format(value))
    
# Create sliders for Commons eta values
eta_slider = ttk.Scale(window, from_=0, to=1, orient=tk.HORIZONTAL, length=300, command=update_label_eta)
eta_slider.set(float(root.find('Commons/eta').text))
eta_slider.grid(row=11, column=0, columnspan=2)  # Place nt_slider in the first row, first column, spanning 2 columns


# ------------------ Create a label widget to display the value -----------------
label_stop_err = tk.Label(window, text="stop_err: 0")
# label_nt.pack()
label_stop_err.grid(row=12, column=0, columnspan=2)  # Place nt_slider in the first row, first column, spanning 2 columns

def update_label_stop_err(value):
    label_stop_err.config(text="stop_err: {}".format(value))
    
# Create sliders for Commons stop_err values
stop_err_slider = ttk.Scale(window, from_=0, to=1, orient=tk.HORIZONTAL, length=300, command=update_label_stop_err)
stop_err_slider.set(float(root.find('Commons/stop_err').text))
stop_err_slider.grid(row=13, column=0, columnspan=2)  # Place nt_slider in the first row, first column, spanning 2 columns

# --------------------- label for the cost sigma ------------------------
label_cost1 = tk.Label(window, text="Exp1_cost_sigma: 0")
label_cost1.grid(row=14, column=0, columnspan=2)  # Place label_sx in the second row, first column, spanning 2 columns

def update_label_cost1(value):
    label_cost1.config(text="Exp1_cost_sigma: {}".format(value))
    
exp1_collision_slider = ttk.Scale(window, from_=1000, to=1e6, orient=tk.HORIZONTAL, length=300, command=update_label_cost1)
exp1_collision_slider.set(float(root.find('Experiment1/cost_sigma').text))

exp1_collision_slider.grid(row=15, column=0, columnspan=2)  

# --------------------- label for the cost sigma ------------------------
label_cost2 = tk.Label(window, text="Exp2_cost_sigma: 0")
label_cost2.grid(row=16, column=0, columnspan=2)  # Place label_sx in the second row, first column, spanning 2 columns

def update_label_cost2(value):
    label_cost2.config(text="Exp2_cost_sigma: {}".format(value))
    
exp2_collision_slider = ttk.Scale(window, from_=1000, to=1e6, orient=tk.HORIZONTAL, length=300, command=update_label_cost2)
exp2_collision_slider.set(float(root.find('Experiment2/cost_sigma').text))

exp2_collision_slider.grid(row=17, column=0, columnspan=2)  

# --------------------- label for the cost sigma ------------------------
label_cost3 = tk.Label(window, text="Exp3_cost_sigma: 0")
label_cost3.grid(row=18, column=0, columnspan=2)  # Place label_sx in the second row, first column, spanning 2 columns

def update_label_cost3(value):
    label_cost3.config(text="Exp3_cost_sigma: {}".format(value))
    
exp3_collision_slider = ttk.Scale(window, from_=1000, to=1e6, orient=tk.HORIZONTAL, length=300, command=update_label_cost3)
exp3_collision_slider.set(float(root.find('Experiment3/cost_sigma').text))

exp3_collision_slider.grid(row=19, column=0, columnspan=2)  

# --------------------- label for the cost sigma ------------------------
label_cost4 = tk.Label(window, text="Exp4_cost_sigma: 0")
label_cost4.grid(row=20, column=0, columnspan=2)  # Place label_sx in the second row, first column, spanning 2 columns

def update_label_cost4(value):
    label_cost4.config(text="Exp4_cost_sigma: {}".format(value))
    
exp4_collision_slider = ttk.Scale(window, from_=1000, to=1e6, orient=tk.HORIZONTAL, length=300, command=update_label_cost4)
exp4_collision_slider.set(float(root.find('Experiment4/cost_sigma').text))

exp4_collision_slider.grid(row=21, column=0, columnspan=2)  

# Create update button
update_button = ttk.Button(window, text="Update XML", command=update_xml)
update_button.grid(row=22, column=0, columnspan=2)

window.mainloop()