function plot_map2(BW,rows,cols)

    colormap([0 0 0; 1 1 1]);  
    imagesc(BW);
    axis equal;
    axis xy;
    grid on;
    xlabel('Column (j)');
    ylabel('Row (i)');
    xlim([0 cols]);
    ylim([0 rows]);
    set(gca, 'YDir', 'reverse')
end