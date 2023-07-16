from flask import (
    Flask,
    render_template,
    request,
    jsonify,
    redirect,
    url_for,
)

import zmq

app = Flask(__name__, static_folder='static', template_folder='templates')

@app.route('/', methods=['GET', 'POST'])
def index():
    jsonData = request.get_json()
    print(jsonData['data'])
    return jsonData['data']

@app.route('/<string:motor_id>/', methods=['GET', 'POST'])
def _motor(motor_id):
    jsonData = request.get_json()
    print(jsonData['data'])
    return jsonData['data']

@app.route('/<string:motor_id>/is_running', methods=['GET', 'POST'])
def _motor_is_running(motor_id: str):

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    return jsonify({
        'data': {
            'motor_id': motor_id,
            'is_running': True
        }
    })

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=True)