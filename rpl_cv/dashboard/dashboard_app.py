import socket
import struct
from typing import Tuple

import cv2
import dash
import numpy as np
import plotly.express as px
from dash import Dash, Input, Output, State, dcc, html
from flask import Flask, Response, current_app


class VideoCamera(object):
    """Camera class that opens camera using OpenCV and gets a JPEG compressed frame from the
        everytime get_frame() is called.
        **Not used anymore after implementing socket connection code. But still kept for
          reference if ever needed again
    Args:
        object (_type_): _description_
    """

    def __init__(self):
        self.video = cv2.VideoCapture(0)  # open webcam

    def __del__(self):
        self.video.release()

    def _resizeImage(self, img, percent):
        width = int(img.shape[1] * percent / 100)
        height = int(img.shape[0] * percent / 100)
        dim = (width, height)
        img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

        return img, dim

    def get_frame(self):
        success, image = self.video.read()
        scale_percent = 50  # percent of original size
        image, _ = self._resizeImage(image, scale_percent)
        image = cv2.flip(image, 1)  # flip horizontally
        ret, jpeg = cv2.imencode(".jpg", image)
        return jpeg.tobytes()


def gen(camera):
    """Deprecated... Used when we wanted camera frames directly from camera not from ros topic / socket conn

    Args:
        camera (_type_): opencv camera

    Yields:
        _type_: HTML content for video frames to be put on the webpage
    """
    while True:
        frame = camera.get_frame()
        yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n\r\n")


def read_bytes(sock, size):
    buf_list = []
    while sum(len(b) for b in buf_list) < size:
        packet = sock.recv(4096)  # receive data in chunks of 4096 bytes (4KB)
        if not packet:
            raise ConnectionError
        buf_list.append(packet)
    return b"".join(buf_list)


def poll_socket(sock_client, addr):
    """Generator function that listens to data coming on the socket connection,
       and then parses the data into the jpeg compressed image and returns the html video content


    Args:
        sock_client (_type_): socket client
        addr (_type_): address of socket connection

    Yields:
        _type_: _description_
    """
    # Each message has a header containing the size of the message
    # header_size is the size of the header itself
    header_size = struct.calcsize("Q")
    buffer = b""

    try:
        while True:
            # Read the header
            buffer += read_bytes(sock_client, header_size - len(buffer))
            # Split the header from the front of the buffer, preserve the rest of the buffer
            header, buffer = buffer[:header_size], buffer[header_size:]
            # Extract the message size from the header
            data_size = struct.unpack("Q", header)[0]

            # Read the data
            buffer += read_bytes(sock_client, data_size - len(buffer))

            # Split the message from the front of the buffer, preserve the rest of the buffer
            img_raw, buffer = buffer[:data_size], buffer[data_size:]
            # Decode the message into an image --- not decoding jpeg compressed img to opencv image so not needed
            # img = cv2.imdecode(np.frombuffer(img_raw, np.byte), cv2.IMREAD_ANYCOLOR)

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + img_raw + b"\r\n\r\n"
            )

    except ConnectionError:
        print("Lost connection from:", addr)
        # sock_client.shutdown(socket.SHUT_RDWR)
        sock_client.close()

    except KeyboardInterrupt:
        print("Got KeyboardInterrupt. Closing socket.")
        sock_client.close()


def create_socket_connection(ip: str, port: int) -> Tuple[object, object, str]:
    """Establishes a socket connection w/ (ip, port). This dashboard app is the server
       and it establishes it with the client.

    Args:
        ip (str): IP Address
        port (int): Port

    Returns:
        Tuple: returns the sock, sock_client, and addr
    """
    # SOCK_STREAM = TCP Connection
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((ip, port))
    sock.listen()
    print(f"Listening at ip:{ip} port:{port}")

    sock_client, addr = sock.accept()
    if not sock_client:
        return
    print(f"Got connection from: {addr}")

    return sock, sock_client, addr


server = Flask(__name__)  # server created to get video frames
app = dash.Dash(__name__, server=server)
(
    server.config["cam_sock"],
    server.config["cam_sock_client"],
    server.config["cam_sock_addr"],
) = create_socket_connection(ip="127.0.0.1", port=9999)

(
    server.config["btn_sock"],
    server.config["btn_sock_client"],
    server.config["btn_sock_addr"],
) = create_socket_connection(ip="127.0.0.1", port=9080)

send_btn_topic = "dash_msg_topic"


@server.route("/video_feed")
def video_feed():
    # region OLD METHOD
    # --- OLD method -- used when not receiving frames from ros topic/socket but instead just
    # getting the frames from camera directly
    # return Response(gen(VideoCamera()),
    #                 mimetype='multipart/x-mixed-replace; boundary=frame')
    # endregion
    return Response(
        poll_socket(
            current_app.config["cam_sock_client"], current_app.config["cam_sock_addr"]
        ),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


app.layout = html.Div(
    [
        # Camera Output
        html.H2("Web Camera Output"),
        html.Img(
            src="/video_feed",
            style={
                "backgroundColor": "darkslategray",
                "color": "lightsteelblue",
                "height": "20%",
                "margin-left": "10px",
                "width": "20%",
                "text-align": "center",
                "display": "inline-block",
            },
        ),
        # Button
        html.Div(dcc.Input(id="input-on-submit", type="text")),
        html.Button("Submit", id="submit-val", n_clicks=0),
        html.Div(
            id="container-button-basic", children="Enter a value and press submit"
        ),
        # Graph
        html.H2("Interactive normal distribution"),
        dcc.Graph(
            id="graph",
            style={
                "backgroundColor": "darkslategray",
                "color": "lightsteelblue",
                "height": "35%",
                "margin-left": "10px",
                "text-align": "center",
                "width": "40%",
                "display": "inline-block",
            },
        ),
        html.H4("Mean:"),
        dcc.Slider(id="mean", min=-3, max=3, value=0, marks={-3: "-3", 3: "3"}),
        html.H4("Standard Deviation:"),
        dcc.Slider(id="std", min=1, max=3, value=1, marks={1: "1", 3: "3"}),
    ]
)


@app.callback(
    Output("container-button-basic", "children"),
    Input("submit-val", "n_clicks"),
    State("input-on-submit", "value"),
)
def button_on_click(n_clicks, value):
    msg = bytes(f"{send_btn_topic}?{value}", "utf-8")
    header = struct.pack("Q", len(msg))

    server.config["btn_sock"].sendall(header + msg)
    return f'Sent message {n_clicks} of " {value}"'


@app.callback(Output("graph", "figure"), Input("mean", "value"), Input("std", "value"))
def display_color(mean, std):
    data = np.random.normal(mean, std, size=500)  # replace with your own data source
    fig = px.histogram(data, range_x=[-10, 10])
    return fig


if __name__ == "__main__":
    # debug=True --> raises an error saying that connection is already in use
    # debug=False OR (debug=True, use_reloader=False) work. Source: https://stackoverflow.com/a/66075333/7359915
    try:
        app.run_server(debug=True, use_reloader=False)
    finally:
        server.config["cam_sock"].close()
        server.config["btn_sock"].close()
        print("Closed socket. Exiting...")
