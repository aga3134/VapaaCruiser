#!/usr/bin/env python
# -*- coding: utf-8 -*-

from flask import Flask, render_template, url_for, request, redirect
from flask_login import LoginManager, UserMixin, login_user, current_user, login_required, logout_user
from flask_wtf.csrf import CSRFProtect
import json
import urllib
from db import SqliteDB

with open("config.json","r") as json_file:
    data = json_file.read()
    config = json.loads(data)

app = Flask(__name__)
app.secret_key = config["sessionKey"]
csrf = CSRFProtect(app)

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

@app.route('/setting', methods=["GET","POST"])  
def setting():  
    if not current_user.is_active:
        return {"status":"fail", "data":"permission denied"}

    sqliteDB = SqliteDB()
    if request.method == "GET":
        data = sqliteDB.GetSetting(current_user.id)
        if data is None:
            return   {"status":"fail", "data":"no data"}
        else:
            return {"status":"ok", "data":data}

    if request.method == "POST": 
        data = {}
        data["userID"] = current_user.id
        data["dataset"] = request.form.get("dataset")
        data["apiKey"] = request.form.get("apiKey")
        sqliteDB.UpdateSetting(data)
        return {"status":"ok","data": data}

@app.route('/path/create', methods=["POST"])  
def pathCreate():
    if request.method == "POST":
        sqliteDB = SqliteDB()
        sqliteDB.CreateNavigationPath(current_user.id, request.form.get("path"))
        return {"status":"ok","data": "path created"}

@app.route('/path/edit', methods=["POST"])  
def pathEdit():
    if request.method == "POST":
        sqliteDB = SqliteDB()
        sqliteDB.EditNavigationPath(current_user.id, request.form.get("path"))
        return {"status":"ok","data": "path updated"}

@app.route('/path/list', methods=["GET"])  
def pathList():
    if request.method == "GET":
        sqliteDB = SqliteDB()
        result = sqliteDB.ListNavigationPath(current_user.id)
        return {"status":"ok","data": result}

@app.route('/path/delete', methods=["POST"])  
def pathDelete():
    if request.method == "POST":
        sqliteDB = SqliteDB()
        sqliteDB.DeleteNavigationPath(current_user.id, request.form.get("pathID"))
        return {"status":"ok","data": "path deleted"}

if __name__ == "__main__":
    app.run( )