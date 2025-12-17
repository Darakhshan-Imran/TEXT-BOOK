import json,urllib.request
url='http://127.0.0.1:8000/auth/register'
data={"email":"test+local@example.com","password":"StrongP@ssw0rd!","first_name":"Test","last_name":"User","auth_provider":"email"}
b=json.dumps(data).encode('utf-8')
req=urllib.request.Request(url,data=b,headers={'Content-Type':'application/json'})
try:
    with urllib.request.urlopen(req,timeout=10) as r:
        print(r.status)
        print(r.read().decode())
except Exception as e:
    import traceback
    traceback.print_exc()
