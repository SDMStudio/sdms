from flask import Flask, url_for
import plotly
import plotly.graph_objects as go
from os import walk
import numpy as np
import pandas as pd

app = Flask(__name__)


@app.route('/')
def index():
    _, _, filenames = next(walk("./"))
    filenames = [file[:-4] for file in filenames if file.endswith(".csv")]
    html = "<h2>Results files</h2><ul>"
    for f in filenames:     
        html += "<li> <a href='/"+ f + "/Trial'>"+ f +"</a></li>"
    html += "</ul>"
    return "{}".format(html)


@app.route('/<path:subpath>/<xaxis>')
def show_subpath(subpath, xaxis="Time"):
    filename = subpath+".csv"
    df = pd.read_csv(filename)
    col_names = list(df.columns)

    html = "<script src='https://cdn.plot.ly/plotly-latest.min.js'></script>"

    for i, name in enumerate(col_names):
        fig = go.Figure()
        fig.add_trace(go.Scatter(y=df[name], x=df[xaxis], mode="lines", name=name, showlegend=True))
        fig.update_layout(title='{} over {}'.format(name, xaxis.lower()),
                          xaxis_title=xaxis,
                          yaxis_title=name,
                          height=500,
                          width=800)
    
        res = plotly.offline.plot(fig, include_plotlyjs=False, output_type='div')
        html += res
    return "{}".format(html)

