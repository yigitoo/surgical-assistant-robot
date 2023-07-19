import requests

for _ in range(100):
	r = requests.post('http://localhost:5632/', json={"cmd_name": "set_angle", "cmd_val": "1,0,0,0,0,0"})
	print(r.text)
