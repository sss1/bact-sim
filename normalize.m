function x = normalize(x)
  if any(x) > 0
    x = x./norm(x);
  end
end
