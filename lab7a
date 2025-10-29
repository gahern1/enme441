import RPi.GPIO as GPIO
import socket
from urllib.parse import parse_qs
import os

GPIO.setmode(GPIO.BCM)
LEDpins = [16, 20, 21]
pwms = []                  # initialize pwm list
for i in LEDpins:
  GPIO.setup(i, GPIO.OUT)
  pwm = GPIO.PWM(i, 1000)
  pwm.start(0)              # start pwm at 0% duty cycle
  pwms.append(pwm)          # add pwm to pwm list

brightness = [0, 0, 0]      # stores brightness data

def html_generator():
  html = f"""<!DOCTYPE html>
<html>
<head>
  <title>LED BRIGHTNESS CONTROLLER</title>
</head>
<body>
  <h3>Brightness Level</h3>
  <form method="POST">
    <input type="range" name="level" min="0" max="100" value="0"><br><br>

    <b>Select LED:</b><br>
    <input type="radio" name="led" value="0" checked> LED 1 ({brightness[0]}%)<br>
    <input type="radio" name="led" value="1"> LED 2 ({brightness[1]}%)<br>
    <input type="radio" name="led" value="2"> LED 3 ({brightness[2]}%)<br><br>
    <input type="submit" value="Change Brightness">
  </form>
</body>
</html>"""
  return html


HOST = ''
PORT = 8080
ip = os.popen("hostname -I").read().split()[0]
print(f"Server running at http://{ip}:{PORT}/")

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
  s.bind((HOST, PORT))
  s.listen(1)

  try:
    while True:
      conn, addr = s.accept()
      with conn:
        request = conn.recv(1024).decode('utf-8', errors='ignore')

        if not request:
          continue

        header, _, body = request.partition('\r\n\r\n')

        if header.startswith('POST'):
          form = parse_qs(body)
          led_idx = int(form.get('led', [0])[0])
          level = int(form.get('level', [0])[0])

          brightness[led_idx] = level
          pwms[led_idx].ChangeDutyCycle(level)

        response_body = html_generator()
        response = (
          "HTTP/1.1 200 OK\r\n"
          "Content-Type: text/html\r\n"
          f"Content-Length: {len(response_body)}\r\n"
          "Connection: close\r\n"
          "\r\n" + 
          response_body
        )
        conn.sendall(response.encode('utf-8'))

  except KeyboardInterrupt:
    print("\nShutting Down...")
  finally:
    for pwm in pwms:
      pwm.stop()
    GPIO.cleanup()
