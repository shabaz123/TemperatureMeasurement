from flask import Flask, send_file, render_template, make_response
import datetime
import StringIO
import random
from time import sleep
from datetime import datetime
import numpy as np
from flask_table import Table, Col

from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.dates import DateFormatter

app = Flask(__name__)

temperature_file="/home/pi/development/therm/rpi/temperature.csv"

@app.route("/")
def hello():
    current_time = str(datetime.now())
    data = np.loadtxt(open("/home/pi/development/therm/rpi/temperature.csv", 'rb'), delimiter=",", dtype='S8,d8,f8', skiprows=1)
    class ItemTable(Table):
        column1 = Col("time")
        column2 = Col("elapsed")
        column3 = Col("temperature")
    class Item(object):
        def __init__(self, column1, column2, column3):
            self.column1 = column1
            self.column2 = column2
            self.column3 = column3
    items = []
    for i in data:
        this_item = Item(i[0],i[1],i[2])
        items.append(this_item)
    table = ItemTable(items)
    return render_template("homepage.html", time=current_time, table=table)

@app.route("/download.csv")
def download():
    return send_file(temperature_file)

@app.route("/display")
def display():
    data = np.loadtxt(open("/home/pi/development/therm/rpi/temperature.csv", 'rb'), delimiter=",", dtype='S8,d8,f8', skiprows=1)
    data = np.array(data.tolist())
    x = data[:,1]
    y = data[:,2]
    fig = Figure()
    ax = fig.add_subplot(111)
    ax.plot(x,y,"ko-")
    
    canvas=FigureCanvas(fig)
    png_output = StringIO.StringIO()
    canvas.print_png(png_output)
    response = make_response(png_output.getvalue())
    response.headers['Content-Type'] = 'image/png'
    return response

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=80, debug=True)
