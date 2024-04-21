import sys
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def draw_rectangles():
    fig, ax = plt.subplots()
    # Set limits and the aspect ratio of the axes
    box_width, box_height = 316, 212
    xlim, ylim = box_width * 1.5, box_height * 1.5
    ax.set_xlim(-xlim, xlim)
    ax.set_ylim(-ylim, ylim)
    ax.set_aspect('equal')

    # Function to draw the central reference box
    def draw_reference_box():
        reference_box = patches.Rectangle((-box_width/2, -box_height/2), box_width, box_height,
                                          linewidth=1, edgecolor='black', facecolor='none')
        ax.add_patch(reference_box)

    draw_reference_box()
    
    # Continuously read from stdin
    try:
        while True:
            line = sys.stdin.readline().strip()
            if line == 'clear':
                plt.cla()  # Clear the plot for new rectangles
                ax.set_xlim(-xlim, xlim)
                ax.set_ylim(-ylim, ylim)
                draw_reference_box()  # Re-add the reference box
            elif line == 'update':
                plt.draw()  # Update the plot to show all changes at once
                plt.pause(0.001)  # Short pause to ensure the plot is updated
            else:
                try:
                    values = line.split(',')
                    left, top, width, height = map(float, values[:4])
                    color = values[4].strip()
                except (ValueError, IndexError) as e:
                    print(f"Skipping invalid input line '{line}': {e}")
                    continue

                # Draw the rectangle
                rect = patches.Rectangle((left, top - height), width, height, linewidth=0,
                                         edgecolor='none', facecolor=color)
                ax.add_patch(rect)
                
                # Adding labels
                ax.text(left, top, f"({left}, {top})", fontsize=8, verticalalignment='top', color='black')
                ax.text(left, top - height/2, f"{height}", fontsize=8, verticalalignment='center', color='black', rotation=90)
                ax.text(left + width/2, top - height, f"{width}", fontsize=8, horizontalalignment='center', color='black')

    except KeyboardInterrupt:
        print("Stopped.")
        sys.exit()

if __name__ == "__main__":
    draw_rectangles()
