import dash
from dash import Dash, dcc, html, Input, Output
import plotly.express as px
import numpy as np
from flask import Flask, Response, current_app
import cv2
import socket
import struct

class VideoCamera(object):
    def __init__(self):
        self.video = cv2.VideoCapture(0)    # open webcam

    def __del__(self):
        self.video.release()

    def _resizeImage(self, img, percent):
        width = int(img.shape[1] * percent / 100)
        height = int(img.shape[0] * percent / 100)
        dim = (width, height)
        img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

        return img, dim

    def get_frame(self):
        success, image = self.video.read()
        scale_percent = 50 # percent of original size
        image, _ = self._resizeImage(image, scale_percent)
        image = cv2.flip(image, 1)   # flip horizontally
        ret, jpeg = cv2.imencode('.jpg', image)
        return jpeg.tobytes()


def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

def read_bytes(sock, size):
    buf_list = []
    while sum(len(b) for b in buf_list) < size:
        packet = sock.recv(4096)
        if not packet:
            raise ConnectionError
        buf_list.append(packet)
    return b''.join(buf_list)

def poll_socket(sock_client, addr):
    # Each message has a header containing the size of the message
    # header_size is the size of the header itself
    header_size = struct.calcsize('Q')
    buffer = b''

    try:
        while True:
            # Read the header
            buffer += read_bytes(sock_client, header_size=len(buffer))
            # Split the header from the front of the buffer, preserve the rest of the buffer
            header, buffer = buffer[:header_size], buffer[header_size:]
            # Extract the message size from the header
            data_size = struct.unpack('Q', header)[0]

            # Read the data
            buffer += read_bytes(sock_client, data_size-len(buffer))

            # Split the message from the front of the buffer, preserve the rest of the buffer
            img_raw, buffer = buffer[:data_size], buffer[data_size:]
            # # Decode the message into an image
            # img = cv2.imdecode(np.frombuffer(img_raw, np.byte), cv2.IMREAD_ANYCOLOR)

            # if callback_fn is not None:
            #     callback_fn(img)
            # TODO: check if you can simply yield img_raw --> decoding
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + img_raw + b'\r\n\r\n')

    except ConnectionError:
        print('Lost connection from:', addr)
        # sock_client.shutdown(socket.SHUT_RDWR)
        sock_client.close()

    except KeyboardInterrupt:
        print('Got KeyboardInterrupt. Closing socket.')
        sock_client.close()

def create_socket_connection(ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((ip, port))
    sock.listen()
    print(f'Listening at ip:{ip} port:{port}')


    sock_client, addr = sock.accept()
    if not sock_client:
        return
    print(f'Got connection from: {addr}')

    return sock, sock_client, addr

server = Flask(__name__)
app = dash.Dash(__name__, server=server)
server.config['sock'], server.config['sock_client'], server.config['sock_addr'] = create_socket_connection(ip="127.0.0.1", port=9090)


@server.route('/video_feed')
def video_feed():
    # return Response(gen(VideoCamera()),
    #                 mimetype='multipart/x-mixed-replace; boundary=frame')
    return Response(poll_socket(current_app.config['sock_client'], current_app.config['sock_addr']),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

app.layout = html.Div([
    html.H2("Web Camera Output"),
    html.Img(src="/video_feed", 
                    style={
                        'backgroundColor':'darkslategray',
                        'color':'lightsteelblue',
                        'height':'20%',
                        'margin-left':'10px',
                        'width':'20%',
                        'text-align':'center',
                        'display':'inline-block'
                        }),

    html.H2('Interactive normal distribution'),
    dcc.Graph(id="graph", style={
                        'backgroundColor':'darkslategray',
                        'color':'lightsteelblue',
                        'height':'35%',
                        'margin-left':'10px',
                        'text-align':'center',
                        'width':'40%',
                        'display':'inline-block'
               }),

    html.H4("Mean:"),
    dcc.Slider(id="mean", min=-3, max=3, value=0, 
               marks={-3: '-3', 3: '3'}),
    html.H4("Standard Deviation:"),
    dcc.Slider(id="std", min=1, max=3, value=1, 
               marks={1: '1', 3: '3'}),
])

@app.callback(Output("graph", "figure"), Input("mean", "value"), Input("std", "value"))
def display_color(mean, std):
    data = np.random.normal(mean, std, size=500) # replace with your own data source
    fig = px.histogram(data, range_x=[-10, 10])
    return fig


if __name__ == '__main__':
    app.run_server(debug=True)
