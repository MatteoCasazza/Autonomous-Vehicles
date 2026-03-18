function node = coords_to_node(i, j, rows, BW)
    node = sub2ind([rows, size(BW,2)], i, j);
end