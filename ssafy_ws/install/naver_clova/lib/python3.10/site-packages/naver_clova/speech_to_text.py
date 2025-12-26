import sys
import requests

client_id = "41xm04n3yf"
client_secret = "4WDoD2GCabWPZoEnwIh7im00HcnAVCIEIS6ciXfG"

lang = "Kor" # 언어 코드 ( Kor, Jpn, Eng, Chn )
url = "https://naveropenapi.apigw.ntruss.com/recog/v1/stt?lang=" + lang

data = open('/home/ssafy/ssafy_ws/src/naver_clova/naver_clova/test.m4a', 'rb')
headers = {
    "X-NCP-APIGW-API-KEY-ID": client_id,
    "X-NCP-APIGW-API-KEY": client_secret,
    "Content-Type": "application/octet-stream"
}

response = requests.post(url,  data=data, headers=headers)
rescode = response.status_code

if(rescode == 200):
    print (response.text)
else:
    print("Error : " + response.text)
