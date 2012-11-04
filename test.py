dt = 0.05
s = 0.0
s1 = 0.0
kp = 0.5
kv = 0.9
prev = 0.0
ax = 1.0
v = 0.0
for line in file('alt1.log').readlines():
   a, r, _ = map(float, line.split(' '))
   a *= -1.0;
   v += a * dt + ((r - prev) * dt - v) * kv
   s += s * a / 2 * dt ** 2 + v + (r - s) * kp
   print r, s, a / 5
   prev = r
