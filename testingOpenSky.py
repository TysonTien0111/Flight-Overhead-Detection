from flask import Flask, jsonify
import requests

app = Flask(__name__)

@app.route("/flights")
def flights():

    url = "https://opensky-network.org/api/states/all?lamin=37.55&lomin=-122.50&lamax=37.70&lomax=-122.30"
    r = requests.get(url)
    data = r.json()

    flights = []

    for state in data["states"][:4]:   # ONLY 4 flights
        callsign = state[1].strip() if state[1] else "UNKNOWN"
        altitude = int(state[7]) if state[7] else 0

        flights.append({
            "callsign": callsign,
            "altitude": altitude
        })

    return jsonify(flights)

app.run(host="0.0.0.0", port=5000)
