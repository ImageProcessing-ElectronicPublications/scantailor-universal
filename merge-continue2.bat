cd C:\Users\drice\scantailor-universal
copy "C:\Users\drice\Downloads\OutputGenerator.cpp" "src\core\filters\output\OutputGenerator.cpp"
git add src/core/filters/output/OutputGenerator.cpp
git commit --no-edit -m "Merge halftone-detection (resolve: keep both text-evidence suppression and halftone detection)"

REM === Resume remaining merges ===
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
