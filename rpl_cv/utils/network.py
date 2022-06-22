import socket
import struct

import cv2
import numpy as np


def simple_video_sender(ip='127.0.0.1', port=8080):
    sock = socket.create_connection((ip, port))

    cap = cv2.VideoCapture(0)
    try:
        while cap.isOpened():
            success, img = cap.read()
            if not success:
                break

            # Quality 0 to 100, default=95
            quality = 95
            img = cv2.resize(img, None, fx=0.5, fy=0.5)
            _, img = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])

            data = img.tobytes()
            header = struct.pack('Q', len(data))

            sock.sendall(header + data)

    except KeyboardInterrupt:
        cap.release()
        sock.close()

def read_bytes(sock, size):
    buf_list = []
    while sum(len(b) for b in buf_list) < size:
        packet = sock.recv(4096)
        if not packet:
            raise ConnectionError
        buf_list.append(packet)
    return b''.join(buf_list)

def simple_video_receiver(ip='', port=8080, callback=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((ip, port))
    sock.listen()
    print(f'Listening at ip:{ip} port:{port}')

    sock_client, addr = sock.accept()
    if not sock_client:
        return
    print(f'Got connection from: {addr}')

    # Each message has a header containing the size of the message
    # header_size is the size of the header itself
    header_size = struct.calcsize('Q')
    buffer = b''
    try:
        while True:
            # Read the header
            buffer += read_bytes(sock_client, header_size-len(buffer))
            # Split the header from the front of the buffer, preserve the rest of the buffer
            header, buffer = buffer[:header_size], buffer[header_size:]
            # Extract the message size from the header
            data_size = struct.unpack('Q', header)[0]

            # Read the data
            buffer += read_bytes(sock_client, data_size-len(buffer))
            # Split the message from the front of the buffer, preserve the rest of the buffer
            img_raw, buffer = buffer[:data_size], buffer[data_size:]
            # Decode the message into an image
            img = cv2.imdecode(np.frombuffer(img_raw, np.byte), cv2.IMREAD_ANYCOLOR)

            if callback is not None:
                callback(img)

            cv2.imshow('Receiving video', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except ConnectionError:
        print('Lost connection from:', addr)
        # sock_client.shutdown(socket.SHUT_RDWR)
        sock_client.close()
    except KeyboardInterrupt:
        print('Got KeyboardInterrupt. Closing socket.')
        # sock_client.shutdown(socket.SHUT_RDWR)
        sock_client.close()
