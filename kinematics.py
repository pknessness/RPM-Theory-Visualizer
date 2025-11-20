import math

veloBRW_changeDT = 1
pos_cart = (0,0.92387953,-0.38268343)
pt = (-0.38268343,0.85355339,0.35355339)

k1 = pos_cart[0]
k2 = pos_cart[1]
k3 = pos_cart[2]

k1_dot = (pt[0] - k1) / veloBRW_changeDT
k2_dot = (pt[1] - k2) / veloBRW_changeDT
k3_dot = (pt[2] - k3) / veloBRW_changeDT

 # Add this epsilon (a tiny number) to your denominators
epsilon = 1e-6 
denominator_sq = (k1*k1 + k2*k2)
denominator_sqrt = math.sqrt(denominator_sq)

# #outer
q1_dot = ((k1*k2_dot) - (k2*k1_dot)) / (denominator_sq + epsilon)
# #inner
q2_dot = -k3_dot / (math.sqrt(denominator_sq + epsilon))

print(-q1_dot)
print(q2_dot)