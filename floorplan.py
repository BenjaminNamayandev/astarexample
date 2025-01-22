import ezdxf
import matplotlib.pyplot as plt
from ezdxf.addons.drawing import RenderContext, Frontend
from ezdxf.addons.drawing.matplotlib import MatplotlibBackend

def dxf_layer_to_png(dxf_path, layer_name):
    doc = ezdxf.readfile(dxf_path) # read the file

    msp = doc.modelspace() # create a model space

    # 3. Filter entities on the given layer
    layer_entities = [e for e in msp] # creating an array of every layer in the dxf file

    # -- Force black lines by setting the color for each entity --
    for entity in layer_entities:
        entity.dxf.color = 1  # Typically black/white

    # 4. Matplotlib setup
    fig, ax = plt.subplots(figsize=(8, 6))
    ctx = RenderContext(doc)
    out = MatplotlibBackend(ax)
    frontend = Frontend(ctx, out)

    # 5. Draw all layer entities at once
    frontend.draw_entities(layer_entities)

    # 6. Auto-scale, set aspect, and hide axes
    ax.autoscale()
    ax.set_aspect("equal")
    plt.axis("off")

    # 7. Save the figure
    plt.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved layer '{layer_name}' to 'output.png' ")

if __name__ == "__main__":
    dxf_path = "FNB1.dxf"
    layer_name = "walls"
    out_path = "walls.png"
    dxf_layer_to_png(dxf_path, layer_name)
