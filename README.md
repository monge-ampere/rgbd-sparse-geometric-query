# RGB-D Sparse Geometric Query

**Project page:**  
<https://monge-ampere.github.io/rgbd-sparse-geometric-query/>

This repository hosts a technical article on a low-level RGB-D vision scheme developed around 2019–2020 for camera calibration, cross-sensor point mapping, and lightweight 3D recovery.

The central idea is simple but important:

> The practical task was not dense full-frame RGB–depth alignment, but sparse mapping of a small number of RGB feature points into the depth image and then into 3D.

Instead of relying on the vendor API for full-frame remapping, the scheme reformulated the problem as a task-specific geometric query:

1. start from one RGB feature point,
2. back-project it into a 3D viewing ray,
3. sample candidate 3D points along a constrained depth interval,
4. project each candidate into the depth camera,
5. validate the candidate by depth consistency,
6. return once a valid hypothesis is found.

In other words, the problem was rewritten from **dense remapping** into a **sparse geometric query**.

---

## Article

The main article is the homepage of this repository:

- [`index.md`](./index.md)

It discusses:

- RGB / Depth / IR geometric relationships
- calibration strategy
- why RGB–IR can be a more stable calibration route than direct RGB–Depth fitting
- sparse cross-sensor point mapping
- ray-based 3D hypothesis generation
- depth-consistency validation
- why this formulation can be much lighter than dense full-frame remapping

---

## Repository structure

```text
.
├── index.md
├── README.md
├── _config.yml
├── _includes/
│   └── head.html
└── assets/
    └── images/
        ├── rgbd-fig1-api-vs-ours.png
        ├── rgbd-fig3-camera-geometry.png
        ├── rgbd-fig4-initialization-model.png
        ├── rgbd-fig5-plane-calibration-board.png
        ├── rgbd-fig6-ir-chessboard-zense.png
        └── rgbd-fig7-ir-vs-rgb-board.jpg
```

## index.md
The main article page.

## _config.yml
Jekyll / GitHub Pages configuration.

## _includes/head.html
Custom head include for MathJax support.

## assets/images/
Figures used in the article.

## Mathematical rendering

This page uses MathJax for LaTeX-style mathematical formulas.

If formulas do not render correctly, check:

- `_includes/head.html` exists
- MathJax script is loaded correctly
- formulas are not accidentally placed inside code blocks

## Local preview

If you want to preview the page locally with Jekyll:

```bash
bundle exec jekyll serve
```

Then open:

```text
http://127.0.0.1:4000/rgbd-sparse-geometric-query/
```

Depending on your local setup, you may need a standard Jekyll environment first.

## Why this repository exists

This work was never turned into a formal paper or a polished product document.  
It was a short-cycle prototype built quickly and not further expanded afterward.

Still, it remains worth documenting because it captures a recurring lesson:

> When the abstraction level of an available API does not match the real task, the right move is often not patching the API workflow, but rewriting the problem at the correct geometric level.

That lesson later reappeared in other work as well:  
choose a better geometric intermediary first, then solve the problem in that structure.

## Notes

This repository is primarily a technical write-up, not a maintained software package.

So the emphasis is on:

- geometric formulation
- system design judgment
- calibration logic
- sparse cross-sensor mapping strategy

rather than on packaging a reusable library.

## License

Unless otherwise stated, all text in this repository is shared for technical communication and personal research presentation.

If you plan to reuse figures or large portions of the article, please cite the repository or ask for permission first.
