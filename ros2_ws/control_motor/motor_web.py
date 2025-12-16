from flask import Flask, render_template , request , url_for ,jsonify , redirect #  確保已加入 redirect 
import pymysql

app = Flask(__name__)

# 資料庫設定
db_settings = {
    "host": "127.0.0.1",
    "port": 3306,
    "user": "root",
    "password": "Charlie000148",
    "db": "motor_project",
    "charset": "utf8"
}

# 1. 新增 GET 路由：負責網頁初始載入和資料顯示 (SELECT) 
#//////////////////////////
#用來設置登入到馬達頁面的條件
#//////////////////////////
@app.route("/")
def password_page():
    # 確保這個路由存在
    return render_template("web_password.html")
#流程是這樣 我已經在sql建好一個資料庫是表示id和password 我把資料輸入到後端 然後再從sql拿出來讓我對比正確的資料
@app.route("/api/password",methods=["POST"])
def password_input():
    data = request.json
    user_id = data.get("user_id")
    user_password = data.get("user_password")
    print(f"DEBUG: 收到登入請求 - ID: {user_id}, PWD: {user_password}")
    is_valid = False
    try:
        conn = pymysql.connect(**db_settings)
        with conn.cursor() as cursor:
            sql = "SELECT password_check FROM login_table WHERE idcard_check = %s"
            cursor.execute(sql, (user_id,))
            result = cursor.fetchone()
    
            if result:
                login_password = result[0] #前面已經把sql的密碼對應id拿出來
                print("查到的密碼:", login_password)
                print("使用者輸入密碼:", user_password)

                # 判斷輸入的密碼是否與資料庫中的密碼匹配
                if login_password == user_password:
                        is_valid = True
                    # Flask 登入成功後清空這筆（等待下一次 RFID
                        cursor.execute("UPDATE logout_table SET logoutdata = ''")
                        cursor.execute("UPDATE loginok_table SET sayok = 'OK'")
                        cursor.execute("UPDATE rfid_table SET rfiddata = '' WHERE id = 1")
                else:
                        cursor.execute("UPDATE loginok_table SET sayok = 'FALL'")
            else:
                cursor.execute("UPDATE loginok_table SET sayok = 'FALL'")
            
        conn.commit()
        conn.close()

    except Exception as ex:
        print("登入驗證資料庫錯誤:", ex)
        # 處理資料庫連線或查詢錯誤時
        return jsonify({"status": "error", "msg": "伺服器連線資料庫失敗，請稍後再試。"}), 500

    # 3. 根據判斷結果決定轉跳
    if is_valid:
        # 判斷為 (True)：就轉跳到馬達狀態頁面
        return jsonify({"status": "success", "redirect_url": url_for('motor_page')})
    else:
        return jsonify({"status": "error", "msg": "ID或密碼輸入錯誤，請重試。"})
@app.route("/api/rfid",methods=["GET"])
def rfid_input():
    try:
        conn = pymysql.connect(**db_settings)
        with conn.cursor() as cursor:
            cursor.execute("SELECT rfiddata FROM rfid_table WHERE id = 1")
            result = cursor.fetchone()
        conn.close()
         # 回傳最新 5 筆資料

        if result:
            latest_rfid = result[0]
        else:
            latest_rfid = "no_data"

        return jsonify({"status": "success", "rfid": latest_rfid })

    except Exception as ex:
        return jsonify({"status": "error", "msg": str(ex)}), 500
@app.route("/api/logout",methods=["POST"])
def log_out():
    try:
        conn = pymysql.connect(**db_settings)
        with conn.cursor() as cursor:
            cursor.execute("UPDATE logout_table SET logoutdata = 'LOGOUT'")
            
            cursor.execute("UPDATE loginok_table SET sayok = ''WHERE id = 1")
            conn.commit()

        conn.close()

        return jsonify({"status": "success", "msg": "使用者以登出"}), 200
    except Exception as ex:
        return jsonify({"status": "error", "msg": str(ex)}), 500
#//////////////
#用來傳輸馬達資料
#//////////////
@app.route("/api/motor")
def motor_page():
    return render_template("web_motor.html")
#刪除資料作用
@app.route("/api/delete", methods=["POST"])
def delete_input():
    try:
        conn = pymysql.connect(**db_settings)
        with conn.cursor() as cursor:
            # 直接清空整個資料表
            cursor.execute("TRUNCATE TABLE temperature_table;")
            cursor.execute("TRUNCATE TABLE lux_table;")
            cursor.execute("TRUNCATE TABLE humidity_table;")
            conn.commit()

        return jsonify({"status": "success", "msg": "資料已清空"})

    except Exception as ex:
        return jsonify({"status": "error", "msg": str(ex)}), 500
#傳輸溫度的後端
@app.route("/api/temp", methods=["POST"])
def temp_input():
    try:
        conn = pymysql.connect(**db_settings)
        with conn.cursor() as cursor:
            sql = "SELECT temdata FROM temperature_table ORDER BY id ASC"
            cursor.execute(sql)
            rows = cursor.fetchall()
        conn.close()
         # 回傳最新 5 筆資料
        data_tem = [r[0] for r in rows][::-1]   # 反轉成正常順序
        return jsonify({"status": "success","data": data_tem})

    except Exception as ex:
        return jsonify({"status": "error", "msg": str(ex)}), 500 #jsonify把 Python 資料 → 轉成 JSON 送給前端
@app.route("/api/put_tem", methods=["GET"])
def show_temdata_page(): # 這是新的函數名稱，用於處理 GET 請求
    data_tem = []
    try:
        conn = pymysql.connect(**db_settings)
        with conn.cursor() as cursor:
            sql = "SELECT temdata FROM temperature_table"
            cursor.execute(sql)
            result = cursor.fetchall()
        conn.close()

        latest = result[0] if result else ""
        data_tem=[chdata[0] for chdata in result] 
        
        # 將列表轉換為單一字串，用換行符號分隔
        tem_content = "\n".join(data_tem)
        
        return render_template("web_motor.html",tem_content=tem_content)

    except Exception as ex:
        print("SQL錯誤:", ex)
        # 如果發生錯誤，傳遞錯誤訊息給前端
        return render_template("web_motor.html", tem_content="資料庫讀取失敗")

#傳輸亮度的後端

@app.route("/api/lux", methods=["POST"])
def lux_input():
    try:
        conn = pymysql.connect(**db_settings)
        with conn.cursor() as cursor:
            sql = "SELECT luxdata FROM lux_table ORDER BY id ASC"
            cursor.execute(sql)
            rows = cursor.fetchall()
        conn.close()
         # 回傳最新 5 筆資料
        data_lux = [r[0] for r in rows][::-1]   # 反轉成正常順序
        return jsonify({"status": "success","data": data_lux})

    except Exception as ex:
        return jsonify({"status": "error", "msg": str(ex)}), 500 #jsonify把 Python 資料 → 轉成 JSON 送給前端
@app.route("/api/put_lux", methods=["GET"])
def show_luxdata_page(): # 這是新的函數名稱，用於處理 GET 請求
    data_lux = []
    try:
        conn = pymysql.connect(**db_settings)
        with conn.cursor() as cursor:
            sql = "SELECT luxdata FROM lux_table"
            cursor.execute(sql)
            result = cursor.fetchall()
        conn.close()

        latest = result[0] if result else ""
        data_lux=[chdata[0] for chdata in result] 
        
        # 將列表轉換為單一字串，用換行符號分隔
        lux_content = "\n".join(data_lux)
        
        return render_template("web_motor.html",lux_content=lux_content)

    except Exception as ex:
        print("SQL錯誤:", ex)
        # 如果發生錯誤，傳遞錯誤訊息給前端
        return render_template("web_motor.html", lux_content="資料庫讀取失敗")
    

#傳輸濕度的後端

@app.route("/api/hum", methods=["POST"])
def hum_input():
    try:
        conn = pymysql.connect(**db_settings)
        with conn.cursor() as cursor:
            sql = "SELECT humdata FROM humidity_table ORDER BY id ASC"
            cursor.execute(sql)
            rows = cursor.fetchall()
        conn.close()
         # 回傳最新 5 筆資料
        data_hum = [r[0] for r in rows][::-1]   # 反轉成正常順序
        return jsonify({"status": "success","data": data_hum})

    except Exception as ex:
        return jsonify({"status": "error", "msg": str(ex)}), 500 #jsonify把 Python 資料 → 轉成 JSON 送給前端

#把轉速寫入sql 
@app.route("/api/speed", methods=["POST"])
def speed_input():
    data = request.json # data 是 Flask 自動幫你把前端送來的 JSON 轉成 Python 變數所以data是固定的 實際上data = {"item": "pen"}
    motor_speed = data["input"]
    try:
        conn = pymysql.connect(**db_settings)
        with conn.cursor() as cursor:
            sql = "INSERT INTO speed_table (speeddata) VALUES (%s)" #INSERT是輸出資料到sql
            cursor.execute(sql, (motor_speed,)) # 執行 SQL，把 item 放進去，例如 ('pen',)
            conn.commit() # 提交變更（一定要）
        conn.close()
        print("SQL 已成功寫入：",motor_speed)
    except Exception as ex:
        print("SQL 錯誤：", ex)
        return {"error": str(ex)}, 500
    
    print("收到前端傳來的：",motor_speed)
    return jsonify({"received_speed": motor_speed})
#清空前面輸入的轉速資料
@app.route("/api/clear_speed", methods=["POST"])
def clear_speed():
    try:
        conn = pymysql.connect(**db_settings)
        with conn.cursor() as cursor:
            cursor.execute("TRUNCATE TABLE speed_table;")  # 清空整張轉速資料表
            conn.commit()
        conn.close()
        print("speed_table 已清空")
        return jsonify({"status": "success", "msg": "speed_table 已清空"})

    except Exception as ex:
        print("清空失敗:", ex)
        return jsonify({"status": "error", "msg": str(ex)}), 500
@app.route("/maintain")
def maintenance_page():
    return render_template("web_maintenance.html")
@app.route("/api/maintenance")
def maintain_page():
    conn = pymysql.connect(**db_settings)
    with conn.cursor() as cursor:
        cursor.execute(
            "SELECT maintaindata FROM maintain_table WHERE id = 1"
        )
        result = cursor.fetchone()
    conn.close()

    return jsonify({
        "maintenance": True if result and result[0] == 1 else False
    })
    




if __name__ == "__main__":
    print("你正在執行這支 motor_web Flask !!!!")
    app.run(host="0.0.0.0", port=5000, debug=True)
#http://192.168.50.36:5000/