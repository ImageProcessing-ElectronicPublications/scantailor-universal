cd C:\Users\drice\scantailor-universal
git checkout master
git branch -D test-all-patches
git push origin --delete test-all-patches
git checkout -b test-all-patches

REM === Dewarping/Deskew patches 1-4 ===
git merge fix-vert-half-correction --no-edit
git merge multiple-seed-lines --no-edit
git merge increase-ransac-iterations --no-edit
git merge mask-pictures-in-deskew --no-edit

REM === TextLine refinement 5-7 ===
git merge inter-snake-repulsion --no-edit
git merge multiscale-snake-refinement --no-edit
git merge adaptive-blur-sigmas --no-edit

REM === More improvements 8-11 ===
git merge adaptive-threshold --no-edit
git merge batch-area-mapping --no-edit
git merge auto-depth-perception --no-edit
git merge centroid-cross-validation --no-edit

REM === Picture detection P1-P10 ===
git merge adaptive-se-picture-detect --no-edit
git merge adaptive-picture-threshold --no-edit
git merge chroma-picture-detection --no-edit
git merge text-evidence-suppression --no-edit
git merge multiscale-gradient --no-edit
git merge halftone-detection --no-edit
git merge contour-picture-zones --no-edit
git merge feathered-mask-transitions --no-edit
git merge scalable-merge-distance --no-edit
git merge sensitivity-aware-threshold --no-edit

REM === Performance P11-P13 ===
git merge openmp-parallelization --no-edit
git merge batch-parallel-processing --no-edit
git merge simd-acceleration --no-edit

REM === Bug fixes and optimization P14-P22 ===
git merge bugfixes-trio --no-edit
git merge cli-batch-parallel --no-edit
git merge smoothing-gaussblur --no-edit
git merge gaussblur-cache-friendly --no-edit
git merge morph-smooth-snapshot --no-edit
git merge polysurface-symmetry --no-edit
git merge thumbnail-downscale --no-edit
git merge voronoi-unify --no-edit
git merge image-cache --no-edit

git push -u origin test-all-patches --force
pause
