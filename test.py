def avg(list):
   s = 0.0
   for i in list:
      s += i
   return s / len(list)

dt = 0.0033333
s = 0.0

kv = 0.01
ki = -0.00001
kp = 0.1

v = 0.0
prev = 0.0
sum = 0.0
avg1 = [0.0] * 60
avg2 = [0.0] * 60
for line in file('alt.log').readlines():
   a, r, _ = map(float, line.split(' '))
   sum += (r - prev) / dt
   v += a * dt + (r - prev) / dt * kv + sum * ki
   s += a / 2.0 * dt ** 2.0 + v * dt + (r - s) * kp
   print r, s
   prev = r
