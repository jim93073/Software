#!/usr/bin/env python
import requests

f = open("smile.png","rb")
imgFile = {"img" : ("smile.png",f,"image/png",{})}
#imgFile = mysql.Binary(img)
#imgFile ={"img" : open("smile.png","rb")}
sendData = {'data':"img",'tokens':'7XWE32L52T03DH61'}

r = requests.post("http://coursesrv.nutn.edu.tw/S10582015/receive.php",data=sendData,files=imgFile)
#data = {'data1':"img",'token':'7XWE32L52T03DH61'}  
#files = {"data2" : open("qrcode_temp.jpeg")}  
#r = requests.get("http://coursesrv.nutn.edu.tw/S10582015/receive.php",params=data,files=files)