#!/usr/bin/env python
import requests
import MySQLdb as mysql
f = open("qrcode_temp.jpeg")
img = f.read()

sendData = {'data1':"img",'data2':mysql.Binary(img),'token':'7XWE32L52T03DH61'}

r = requests.get("http://coursesrv.nutn.edu.tw/S10582015/receive.php",params=sendData,timeout=5)
#data = {'data1':"img",'token':'7XWE32L52T03DH61'}  
#files = {"data2" : open("qrcode_temp.jpeg")}  
#r = requests.get("http://coursesrv.nutn.edu.tw/S10582015/receive.php",params=data,files=files)