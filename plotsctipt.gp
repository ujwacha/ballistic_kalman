# Set labels and titles
set xlabel "X-axis"
set ylabel "Y-axis"
set zlabel "Z-axis"
set title "3D Projectile Path"

# Configure grid and viewing angle
set grid
# set view 60, 30  # Adjust the viewing angle

# Set terminal and output file
# set terminal pngcairo size 800,600
# set output 'projectile_path.png'
set terminal qt size 1920,1080
# Plot the data
# splot 'data.txt' using 1:2:3 with lines lc rgb "blue" lw 2 title "Path"

splot \
      'data.txt' index 0 every ::0::9 using 1:2:3 with lines lc rgb "red" lw 2 title "Start", \
      'data.txt' index 0 every ::10::999 using 1:2:3 with lines lc rgb "blue" lw 2 title "Path", \
      'data.txt' index 0 every ::0::999 using 4:5:6 with lines lc rgb "orange" lw 3 title "Kalman", \
      'data.txt' index 0 every ::0::999 using 7:8:9 with lines lc rgb "green" lw 3 title "noise"

pause -1 "Press any key to close the window..."
# Save and close the output
#unset output

