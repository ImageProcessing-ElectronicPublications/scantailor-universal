cd C:\Users\drice\scantailor-universal
copy "C:\Users\drice\Downloads\TextLineRefiner.cpp" "src\dewarping\TextLineRefiner.cpp"
git add src/dewarping/TextLineRefiner.cpp
git commit --no-edit -m "Merge multiscale-snake-refinement (resolve: combine multi-scale passes with repulsion pointers)"

REM === Resume remaining merges ===
git merge adaptive-blur-sigmas --no-edit
git merge adaptive-threshold --no-edit
git merge batch-area-mapping --no-edit
git merge auto-depth-perception --no-edit
git merge centroid-cross-validation --no-edit
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
git merge openmp-parallelization --no-edit
git merge batch-parallel-processing --no-edit
git merge simd-acceleration --no-edit
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
