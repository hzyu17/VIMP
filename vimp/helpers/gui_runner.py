import xml.etree.ElementTree as ET
import tkinter as tk
from tkinter import ttk

    
# Function to update the XML values
def update_xml():
    new_commons_val = commons_slider.get()  # Get value from commons slider
    new_experiment1_val = experiment1_slider.get()  # Get value from experiment1 slider

    # Update Commons values
    root.find('Commons/nt').text = str(new_commons_val)
    root.find('Commons/eps_sdf').text = str(new_commons_val)
    root.find('Commons/speed').text = str(new_commons_val)
    root.find('Commons/sig0').text = str(new_commons_val)
    root.find('Commons/sigT').text = str(new_commons_val)
    root.find('Commons/eta').text = str(new_commons_val)
    root.find('Commons/stop_err').text = str(new_commons_val)
    root.find('Commons/max_iter').text = str(new_commons_val)

    # Update Experiment1/start_pos/x value
    root.find('Experiment1/start_pos/x').text = str(new_experiment1_val)

    # Save updated XML to a file
    tree.write('/home/hongzhe/git/VIMP/vimp/helpers/example.xml')
    print('XML values updated and saved.')

# Load the XML file
tree = ET.parse('/home/hongzhe/git/VIMP/vimp/helpers/example.xml')
root = tree.getroot()

# Create tkinter window
window = tk.Tk()
window.title("XML Value Updater")

# Create a label widget to display the value of the slider
label_nt = tk.Label(window, text="Value: 0")
# label_nt.pack()
label_nt.grid(row=0, column=0, columnspan=2)  # Place commons_slider in the first row, first column, spanning 2 columns

def update_label_nt(value):
    label_nt.config(text="Value: {}".format(value))
    
# Create sliders for Commons and Experiment1/start_pos/x values
commons_slider = ttk.Scale(window, from_=0, to=100, orient=tk.HORIZONTAL, length=300, command=update_label_nt)
commons_slider.set(float(root.find('Commons/nt').text))
commons_slider.grid(row=1, column=0, columnspan=2)  # Place commons_slider in the first row, first column, spanning 2 columns

label_sx = tk.Label(window, text="Value: 0")
label_sx.grid(row=2, column=0, columnspan=2)  # Place label_sx in the second row, first column, spanning 2 columns

def update_label_sx(value):
    label_sx.config(text="Experiment1/start_pos/x Value: {}".format(value))
    
experiment1_slider = ttk.Scale(window, from_=-20, to=20, orient=tk.HORIZONTAL, length=300, command=update_label_sx)
experiment1_slider.set(float(root.find('Experiment1/start_pos/x').text))

experiment1_slider.grid(row=3, column=0, columnspan=2)  # Place experiment1_slider in the third row, first column, spanning 2 columns


# Create update button
update_button = ttk.Button(window, text="Update XML", command=update_xml)
update_button.grid(row=4, column=0, columnspan=2)

window.mainloop()