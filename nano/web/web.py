#!/usr/bin/env python
# -*- coding: utf-8 -*-

from flask import Flask, render_template, url_for, request, redirect
from flask_login import LoginManager, UserMixin, login_user, current_user, login_required, logout_user
import json
import urllib

with open("config.json","r") as json_file:
    data = json_file.read()
    config = json.loads(data)

app = Flask(__name__)
app.secret_key = config["sessionKey"]

#login related
login_manager = LoginManager(app) 

class User(UserMixin):
    pass

@login_manager.user_loader  
def user_loader(username):  
    if username != config["login"]["username"]:  
        return  
    user = User()  
    user.id = username  
    return user

#route
@app.route("/")
def home():
    if not current_user.is_active:
        return redirect("/login")
    return render_template("index.html",config=config)

@app.route("/login", methods=['GET', 'POST'])  
def login():  
    if request.method == 'GET':  
           return render_template("login.html",config=config)

    username = request.form["username"]
    password = request.form["password"]
    if username == config["login"]["username"] and password == config["login"]["password"]:
        user = User()
        user.id = username
        login_user(user)  
        return redirect("/")
    
    return redirect("/login?message="+urllib.pathname2url("登入失敗"))

@app.route('/logout')  
def logout():  
    logout_user()  
    return redirect("/login")

if __name__ == "__main__":
    app.run( )