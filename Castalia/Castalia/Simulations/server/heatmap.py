import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import seaborn as sns
import json
import sys

def heatmap(data, row_labels, col_labels, ax=None,
            cbar_kw={}, cbarlabel="", **kwargs):
	# sphinx_gallery_thumbnail_number = 2

    """
    Create a heatmap from a numpy array and two lists of labels.

    Arguments:
        data       : A 2D numpy array of shape (N,M)
        row_labels : A list or array of length N with the labels
                     for the rows
        col_labels : A list or array of length M with the labels
                     for the columns
    Optional arguments:
        ax         : A matplotlib.axes.Axes instance to which the heatmap
                     is plotted. If not provided, use current axes or
                     create a new one.
        cbar_kw    : A dictionary with arguments to
                     :meth:`matplotlib.Figure.colorbar`.
        cbarlabel  : The label for the colorbar
    All other arguments are directly passed on to the imshow call.
    """

    if not ax:
        ax = plt.gca()

    # Plot the heatmap
    im = ax.imshow(data, vmin=0, vmax=400, **kwargs)

    # Create colorbar
    cbar = ax.figure.colorbar(im, ax=ax, **cbar_kw)
    cbar.ax.set_ylabel(cbarlabel, rotation=-90, va="bottom")

    # We want to show all ticks...
    # ax.set_xticks(np.arange(data.shape[1]))
    # ax.set_yticks(np.arange(data.shape[0]))
    # ... and label them with the respective list entries.
    # ax.set_xticklabels(col_labels)
    # ax.set_yticklabels(row_labels)
    ax.tick_params(axis=u'both', which=u'both',length=0)

    # Let the horizontal axes labeling appear on top.
    ax.tick_params(top=True, bottom=False,
                   labeltop=True, labelbottom=False)

    # Rotate the tick labels and set their alignment.
    # plt.setp(ax.get_xticklabels(), rotation=-30, ha="right",
    #          rotation_mode="anchor")

    # Turn spines off and create white grid.
    # for edge, spine in ax.spines.items():
    #     spine.set_visible(False)

    # ax.set_xticks(np.arange(data.shape[1]+1)-.5, minor=True)
    # ax.set_yticks(np.arange(data.shape[0]+1)-.5, minor=True)
    # ax.grid(which="minor", color="w", linestyle='-', linewidth=3)
    # ax.tick_params(which="minor", bottom=False, left=False)

    return im, cbar


def annotate_heatmap(im, data=None, valfmt="{x:.2f}",
                     textcolors=["black", "white"],
                     threshold=None, **textkw):
    """
    A function to annotate a heatmap.

    Arguments:
        im         : The AxesImage to be labeled.
    Optional arguments:
        data       : Data used to annotate. If None, the image's data is used.
        valfmt     : The format of the annotations inside the heatmap.
                     This should either use the string format method, e.g.
                     "$ {x:.2f}", or be a :class:`matplotlib.ticker.Formatter`.
        textcolors : A list or array of two color specifications. The first is
                     used for values below a threshold, the second for those
                     above.
        threshold  : Value in data units according to which the colors from
                     textcolors are applied. If None (the default) uses the
                     middle of the colormap as separation.

    Further arguments are passed on to the created text labels.
    """

    if not isinstance(data, (list, np.ndarray)):
        data = im.get_array()

    # Normalize the threshold to the images color range.
    if threshold is not None:
        threshold = im.norm(threshold)
    else:
        threshold = im.norm(data.max())/2.

    # Set default alignment to center, but allow it to be
    # overwritten by textkw.
    kw = dict(horizontalalignment="center",
              verticalalignment="center")
    kw.update(textkw)

    # Get the formatter in case a string is supplied
    if isinstance(valfmt, str):
        valfmt = matplotlib.ticker.StrMethodFormatter(valfmt)

    # Loop over the data and create a `Text` for each "pixel".
    # Change the text's color depending on the data.
    texts = []
    for i in range(data.shape[0]):
        for j in range(data.shape[1]):
            kw.update(color=textcolors[im.norm(data[i, j]) > threshold])
            text = im.axes.text(j, i, valfmt(data[i, j], None), **kw)
            texts.append(text)

    return texts




# im, cbar = heatmap(harvest, vegetables, farmers, ax=ax,
#                    cmap="Blues", cbarlabel="harvest [t/year]")

#texts = annotate_heatmap(im, valfmt="{x:.1f} t")
# rect = patches.Rectangle((9.5,17.5),20.5,12.5,linewidth=1,edgecolor='r',facecolor='none')

# Add the patch to the Axes
# ax.add_patch(rect)
# ax.add_patch(patches.Rectangle((10, 18), 30, 30, fill=False, edgecolor='blue', lw=3))

# fig.tight_layout()
fieldWidth = float(sys.argv[2])
fieldHeight = float(sys.argv[3])
cellWidth = float(sys.argv[4])
cellHeight = float(sys.argv[5])
nCol = int(fieldWidth/cellWidth)+1
nRow = int(fieldHeight/cellHeight)+1
arr = []
for b in range(nRow):
    arr.append([])
    for a in range(nCol):
        arr[b].append(0)

jsonFile = sys.argv[1]
f = open(jsonFile, "r") 
datas = json.loads(f.read())
for data in datas:
    col = int(data["x"]/cellWidth)
    row = int(data["y"]/cellHeight)
    arr[row][col] = data["totalPacketReceived"]

data = np.array(arr)

mpl.rcParams['hatch.linewidth'] = 0.1
ax = sns.heatmap(data, cmap="Reds", mask=data < 0, square=True, vmin=0, vmax=1000)
ax.tick_params(axis=u'both', which=u'both',length=0)
# ax.set_facecolor('black')
# ax.add_patch(patches.Rectangle((10, 18), 21, 13, fill=False, edgecolor="red", hatch='xxxx'))
plt.show()