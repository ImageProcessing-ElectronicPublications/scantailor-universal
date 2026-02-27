@echo off
setlocal enabledelayedexpansion
cd C:\Users\drice\scantailor-universal

echo ============================================
echo  ScanTailor Universal - Auto Merge Script
echo ============================================
echo.

REM Abort any in-progress merge
git merge --abort 2>nul

set FAIL_COUNT=0
set SUCCESS_COUNT=0

REM === Merge each branch, auto-resolve conflicts ===

call :merge_branch feathered-mask-transitions
call :merge_branch scalable-merge-distance
call :merge_branch sensitivity-aware-threshold
call :merge_branch openmp-parallelization
call :merge_branch batch-parallel-processing
call :merge_branch simd-acceleration
call :merge_branch bugfixes-trio
call :merge_branch cli-batch-parallel
call :merge_branch smoothing-gaussblur
call :merge_branch gaussblur-cache-friendly
call :merge_branch morph-smooth-snapshot
call :merge_branch polysurface-symmetry
call :merge_branch thumbnail-downscale
call :merge_branch voronoi-unify
call :merge_branch image-cache

echo.
echo ============================================
echo  Results: !SUCCESS_COUNT! succeeded, !FAIL_COUNT! failed
echo ============================================
echo.

echo Pushing to origin...
git push -u origin test-all-patches --force

echo.
pause
goto :eof

:merge_branch
echo.
echo === Merging %1 ===
git merge %1 --no-edit >nul 2>&1
if !errorlevel! equ 0 (
    echo   OK: %1 merged cleanly
    set /a SUCCESS_COUNT+=1
    goto :eof
)
echo   CONFLICT detected - auto-resolving...
python auto_resolve.py --all
git add -A >nul 2>&1
git commit --no-edit -m "Merge %1 (auto-resolved)" >nul 2>&1
if !errorlevel! equ 0 (
    echo   OK: %1 merged with auto-resolve
    set /a SUCCESS_COUNT+=1
) else (
    echo   FAILED: %1 - aborting this merge
    git merge --abort 2>nul
    set /a FAIL_COUNT+=1
)
goto :eof
