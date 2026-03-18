function [i, j] = node_to_coords(node, rows, BW)
    [i, j] = ind2sub([rows, size(BW,2)], node);
end
