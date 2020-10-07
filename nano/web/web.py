from flask import Flask, render_template
import json

with open("config.json","r") as json_file:
    data = json_file.read()
    config = json.loads(data)

app = Flask(__name__)

@app.route("/")
def home():
    return render_template("index.html",config=config)

if __name__ == "__main__":
    app.run( port=config["web"]["port"])