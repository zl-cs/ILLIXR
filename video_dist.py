from pathlib import Path
import multiprocessing
import random
from tqdm import tqdm
from typing import Tuple, cast, Mapping
import numpy as np
import pandas as pd
import cv2
from skimage.metrics import structural_similarity
from skimage.io import imread, imsave
import charmonium.cache as ch_cache
from common import Results


cache_path = Path() / ".cache" / "pathsv2"
cache_path.mkdir(parents=True, exist_ok=True)
# pool = multiprocessing.Pool()
class NotParallelPool:
    def imap(self, fn, args_list):
        return (fn(args) for args in args_list)
pool = NotParallelPool()


@ch_cache.decor(ch_cache.FileStore.create(cache_path))
def compute_ssim(results: Results) -> pd.DataFrame:
    frames = results.frames.copy()
    frames["ssim"] = list(tqdm(
        pool.imap(compute_one_ssim, frames[["path", "gt_path"]].to_numpy()),
        desc="compute_ssim",
        total=len(frames),
        unit="frames",
    ))
    return frames


def compute_one_ssim(paths: Tuple[Path, Path]) -> float:
    imgs = tuple(map(imread, paths))
    if random.randint(0, 200) == 0:
        ssim_dir = cache_path / "log"
        ssim_dir.mkdir(exist_ok=True)
        with (ssim_dir / "output.log").open("a") as f:
            ssim_path = ssim_dir / f"{random.randint(0, 2**256):x}.png"
            ssim, ssim_img = structural_similarity(
                imgs[0],
                imgs[1],
                multichannel=True,
                full=True,
            )
            ssim_img = ((ssim_img - ssim_img.min()) * 255 / ssim_img.max()).astype(np.uint8)
            imsave(ssim_path, ssim_img)
            f.write(f"""
ssim
{paths[0]}
{paths[1]}
{ssim_path}
{ssim}

""")
        return cast(float, ssim)
    else:
        return cast(float, structural_similarity(imgs[0], imgs[1], multichannel=True))


@ch_cache.decor(ch_cache.FileStore.create(cache_path))
def compute_feature_dist(results: Results) -> pd.DataFrame:
    frames = results.frames.copy()
    scores = list(tqdm(
        pool.imap(compute_one_feature_dist, frames[["path", "gt_path"]].to_numpy()),
        desc="compute_feature_dist",
        total=len(frames),
        unit="frames",
    ))
    frames[list(scores[0].keys())] = 0.0
    for i, score in enumerate(scores):
        for score_key, score_val in score.items():
            frames.loc[frames.index[i], score_key] = score_val
    return frames

def compute_one_feature_dist(path_pair: Tuple[Path, Path]) -> Mapping[str, np.float64]:
    img_pair = tuple(map(imread, path_pair))
    feature_detector = cv2.SIFT_create()
    keypoints_pair, descriptors_pair= list(zip(*(
        feature_detector.detectAndCompute(img_pair[0], None),
        feature_detector.detectAndCompute(img_pair[1], None),
    )))
    feature_matcher = cv2.BFMatcher()
    matches = feature_matcher.match(*descriptors_pair)
    distances = np.array([match.distance for match in matches])

    # if random.randint(200, 0) == 0:
    #     fd_dir = cache_path / "log"
    #     fd_dir.mkdir(exist_ok=True)
    #     with (fd_dir / "output.log", "a"):
    #         fd_path = fd_dir / f"{random.randint(0, 2**256):x}.png"
    #         fd, fd_img = structural_similarity(
    #             imread(path),
    #             imread(gt_frames.iloc[gt_frame_no]["path"]),
    #             multichannel=True,
    #             full=True,
    #         )
    #         fd_img = ((fd_img - fd_img.min()) * 255 / fd_img.max()).astype(np.uint8)
    #         imsave(fd_path, fd_img)
    #         f.write(f"""
    #         fd
    #         {row["path"]}
    #         {row["gt_path"]}
    #         {fd_path}
    #         {fd}
    #         """)

    return dict(
        median_feature_dist=np.median(distances) if len(distances) else 0,
        mean_feature_dist=np.mean(distances) if len(distances) else 0,
        max_feature_dist=np.max(distances) if len(distances) else 0,
    )


def compute_video_dists(results: Results) -> Results:
    results.frames = compute_ssim(results)
    results.frames = compute_feature_dist(results)
    return results


video_dists = [
    "ssim",
    "median_feature_dist",
    "mean_feature_dist",
    "max_feature_dist",
]
