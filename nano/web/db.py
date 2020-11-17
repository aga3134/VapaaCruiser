import sqlite3
import json
import uuid

class SqliteDB:
    def __init__(self):
        print(sqlite3.version)
        self.conn = sqlite3.connect("vapaa_cruiser.db")
        self.CreateTable()

    def __del__(self):
        self.conn.close()

    def CreateTable(self):
        c = self.conn.cursor()
        c.execute('''CREATE TABLE IF NOT EXISTS Setting
            (userID TEXT PRIMARY KEY NOT NULL,
            dataset TEXT,
            apiKey TEXT);
        ''')
        c.execute('''CREATE TABLE IF NOT EXISTS NavigationPath
            (id TEXT PRIMARY KEY,
            userID TEXT,
            path TEXT);
        ''')
        self.conn.commit()

    def GetSetting(self, userID):
        c = self.conn.cursor()
        cmd = "SELECT userID, dataset,apiKey FROM Setting WHERE userID='%s';" % (userID)
        result = c.execute(cmd).fetchone()
        if result is None:
            return None
        else:
            return {
                "userID": result[0],
                "dataset": result[1],
                "apiKey": result[2]
            }

    def UpdateSetting(self, data):
        found = self.GetSetting(data["userID"])
        c = self.conn.cursor()
        if found == None:
            cmd = '''INSERT INTO Setting (userID, dataset,apiKey) VALUES('%s', '%s','%s');''' % (data["userID"],data["dataset"],data["apiKey"])
        else:
            cmd = "UPDATE Setting SET dataset='%s', apiKey='%s' WHERE userID='%s';" % (data["dataset"],data["apiKey"],data["userID"])
        c.execute(cmd)
        self.conn.commit()

    def CreateNavigationPath(self,userID,path):
        c = self.conn.cursor()
        id = str(uuid.uuid4())
        path = json.loads(path)
        path["id"] = id
        cmd = '''INSERT INTO NavigationPath (id, userID, path) VALUES('%s','%s','%s');''' % (id,userID,json.dumps(path))
        c.execute(cmd)
        self.conn.commit()

    def EditNavigationPath(self,userID,path):
        c = self.conn.cursor()
        path = json.loads(path)
        cmd = "UPDATE NavigationPath SET path='%s' WHERE userID='%s' AND id='%s';" % (json.dumps(path),userID,path["id"])
        c.execute(cmd)
        self.conn.commit()

    def ListNavigationPath(self,userID):
        c = self.conn.cursor()
        cmd = "SELECT * FROM NavigationPath WHERE userID='%s';" % (userID)
        result = c.execute(cmd).fetchall()
        arr = []
        for row in result:
            p = {
                "id": row[0],
                "userID": row[1],
                "path": json.loads(row[2])
            }
            arr.append(p)
        return arr

    def DeleteNavigationPath(self,userID,pathID):
        c = self.conn.cursor()
        cmd = "DELETE from NavigationPath WHERE userID='%s' AND id='%s';" % (userID,pathID)
        c.execute(cmd)
        self.conn.commit()