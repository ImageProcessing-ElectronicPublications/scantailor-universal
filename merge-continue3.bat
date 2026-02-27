cd C:\Users\drice\scantailor-universal
copy "C:\Users\drice\Downloads\resolve_conflict3.py" .
python resolve_conflict3.py
del resolve_conflict3.py
git add src/core/filters/output/OutputGenerator.cpp src/core/filters/output/OutputGenerator.h
git commit --no-edit -m "Merge contour-picture-zones (resolve: keep both boostMaskWithChroma and contourize)"

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
