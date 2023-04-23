function next = simulate(x, u, dist, dt, noise)
  if dist == 0
      dist = [0,0]';
  end
  if noise == 0
      noise = [0,0]';
  end
  dx = zeros(length(x),1);
  dx(1) = cos(x(3)) + dist(1);
  dx(2) = sin(x(3)) + dist(2);
  dx(3) = u;
  next = x + dt.*(dx + [noise; 0]);
end