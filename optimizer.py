import numpy as np

def gradient_fd(f, x):
  num_variables = len(x)
  out = [ ]

  f_0 = f(x)
  delta_x = 0.0001

  for i in range(num_variables):
    x_copy = np.copy(x)
    x_copy[i] = x_copy[i] + delta_x
    f_delta = f(x_copy)
    out.append( (f_delta - f_0) / delta_x )

  return np.array(out)

def hessian_fd(f, x):
  num_variables = len(x)
  out = [ ]

  f_0 = gradient_fd(f, x)
  delta_x = 0.0001

  for i in range(num_variables):
    x_copy = np.copy(x)
    x_copy[i] = x_copy[i] + delta_x
    f_delta = gradient_fd(f, x_copy)
    out.append( (f_delta - f_0) / delta_x )

  return np.array(out)

def model(f, x, p):
  return f(x) + np.dot(gradient_fd(f, x), p) + 0.5 * p @ hessian_fd(f, x) @ p

def cauchy_point(f, x, r):
  d = gradient_fd(f, x)
  ps = -r * d / np.linalg.norm(d)
  z = hessian_fd(f, x)
  m = d @ z @ d
  if m <= 0:
    tau = 1
  else:
    tau = min(1, np.linalg.norm(d)**3 / (r * m))
  return tau * ps

def trust_region(f, xk, rk = 0.5, eta = 0.2, r_hat = 10, tolerance = 0.001):
  iteration = 0
  while f(xk) >= tolerance and iteration < 500:
    pk = cauchy_point(f, xk, rk)
    rhok = (f(xk) - f(xk + pk)) / (model(f, xk, np.zeros_like(xk)) - model(f, xk, pk) + 1e-6)

    if rhok < 0.25:
      rk *= 0.25
    elif rhok > 0.75 and np.linalg.norm(pk) < rk:
      rk = min(2 * rk, r_hat)

    if rhok > eta:
      xk = xk + pk

    iteration += 1
  print("Iterations: ", iteration)

  return xk