import sqlite3

class SqliteDB:
    def __init__(self):
        print(sqlite3.version)
        self.conn = sqlite3.connect("vapaa_cruiser.db")
        self.CreateTable()

    def __del__(self):
        self.conn.close()

    def CreateTable(self):
        c = self.conn.cursor()
        c.execute('''CREATE TABLE IF NOT EXISTS setting
            (userID TEXT PRIMARY KEY NOT NULL,
            dataset TEXT,
            apiKey TEXT);
        ''')
        c.execute('''CREATE TABLE IF NOT EXISTS navigation_path
            (id INTEGER PRIMARY KEY AUTOINCREMENT,
            userID TEXT,
            info TEXT);
        ''')
        self.conn.commit()

    def GetSetting(self, userID):
        c = self.conn.cursor()
        cmd = "SELECT userID, dataset,apiKey FROM setting WHERE userID='%s';" % (userID)
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
            cmd = '''INSERT INTO setting (userID, dataset,apiKey)VALUES('%s', '%s','%s');''' % (data["userID"],data["dataset"],data["apiKey"])
        else:
            cmd = "UPDATE setting SET dataset='%s', apiKey='%s' WHERE userID='%s';" % (data["dataset"],data["apiKey"],data["userID"])
        c.execute(cmd)
        self.conn.commit()