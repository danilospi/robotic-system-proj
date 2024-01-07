import json
import threading
import requests
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse
from io import BytesIO

class PhidiasHTTPServer_RequestHandler(BaseHTTPRequestHandler):
    ui = None
    port = 0

    def do_GET(self):
        self.send_response(500)
        return

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        body = self.rfile.read(content_length)
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        payload = json.loads(body.decode())
        # payload = { 'from' : source,
        #             'to': agent_name,
        #             'data' : ['belief', [ belief.name(), belief.string_terms() ] ] }
        response = process_incoming_request(PhidiasHTTPServer_RequestHandler.ui, self.client_address[0], payload)

        body = json.dumps(response)
        response = BytesIO()
        response.write(body.encode())
        self.wfile.write(response.getvalue())

    def log_message(self, format, *args):
        return


def send_belief_http(agent_name, destination, belief, terms, source):
    parsed_url = urlparse("//" + destination)
    if parsed_url.hostname is None:
        raise InvalidDestinationException() # type: ignore
    port = parsed_url.port
    if port is None:
        port = 6565

    payload = {'from': source,
               'net-port': PhidiasHTTPServer_RequestHandler.port,
               'to': agent_name,
               'data': ['belief', [belief, terms]]}

    json_payload = json.dumps(payload)
    # print(json_payload)
    new_url = "http://" + parsed_url.hostname + ":" + str(port)
    r = requests.post(new_url, data=json_payload)
    reply = json.loads(r.text)
    if reply['result'] != "ok":
        print("Messaging Error: ", reply)


def server_thread_http(ui, port):
    server_address = ('', port)
    PhidiasHTTPServer_RequestHandler.port = port
    PhidiasHTTPServer_RequestHandler.ui = ui
    httpd = HTTPServer(server_address, PhidiasHTTPServer_RequestHandler)
    print("")
    print("\tPHIDIAS Messaging Server is running at port ", port)
    print("")
    print("")
    # print(httpd.socket)
    httpd.serve_forever()
    server_thread() # type: ignore


#
# MAIN server startup function
#
def start_message_server_http(ui, port=6566):
    t = threading.Thread(target=server_thread_http, args=(ui, port,))
    t.daemon = True
    t.start()
    return t


# protocol-independent

def process_incoming_request(ui, from_address, payload):
    response = {'result': 'err',
                'reason': 'Malformed HTTP payload',
                'data': payload}
    if 'from' in payload.keys():
        if 'net-port' in payload.keys():
            if 'to' in payload.keys():
                if 'data' in payload.keys():
                    # format is valid
                    _from = payload['from']
                    _to = payload['to']
                    _data = payload['data']
                    _net_port = payload['net-port']
                    if _net_port == 0:
                        _from = _from + "@<unknown>"
                    else:
                        _from = _from + "@" + from_address + ":" + repr(_net_port)
                    if _to == 'robot':
                        if _data[0] == 'belief':
                            [Name, Terms] = _data[1]
                            if Name == 'scan_nearest_position':
                                ui.set_from(_from)
                                ui.scan_nearest_position(*Terms)
                            elif Name == 'go_to_nearest_block':
                                ui.set_from(_from)
                                ui.go_to_nearest_block(*Terms)
                            elif Name == 'go_to_bucket':
                                ui.set_from(_from)
                                ui.go_to_bucket(*Terms)
                            elif Name == 'sense_color':
                                ui.set_from(_from)
                                ui.sense_color(*Terms)
                            elif Name == 'generate_blocks':
                                ui.set_from(_from)
                                ui.generate_blocks(*Terms)
                            elif Name == 'go_to_start':
                                ui.set_from(_from)
                                ui.go_to_start(*Terms)
                            response = {'result': 'ok'}
                        else:
                            response = {'result': 'err',
                                        'reason': 'Invalid verb',
                                        'data': _data}
                    else:
                        response = {'result': 'err',
                                    'reason': 'Destination agent not found',
                                    'data': _to}
    return response


class Messaging:
    @classmethod
    def parse_destination(cls, agent_name):
        at_pos = agent_name.find("@")
        if at_pos < 0:
            return None, None
        else:
            agent_local_name = agent_name[:at_pos]
            site_name = agent_name[at_pos + 1:]
            return agent_local_name, site_name

    @classmethod
    def send_belief(cls, destination, belief, terms, source):
        (agent_name, destination) = Messaging.parse_destination(destination)
        send_belief_http(agent_name, destination, belief, terms, source)
