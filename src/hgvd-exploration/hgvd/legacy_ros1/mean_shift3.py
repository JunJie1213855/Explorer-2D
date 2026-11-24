from typing import Optional, Set, Tuple
import numpy as np
from sklearn.calibration import Parallel, delayed
from sklearn.cluster import estimate_bandwidth, get_bin_seeds
from sklearn.neighbors import NearestNeighbors
from utils import ConnectivityCache


class MyNearestNeighbors:
    """带连通性约束的最近邻居搜索"""

    def __init__(
        self,
        radius: int,
        map: np.ndarray,
        connectivity_cache: ConnectivityCache,
        n_jobs: Optional[int] = None,
    ) -> None:
        """
        带连通性约束的最近邻居搜索

        Parameters
        ----------
        radius: int 邻居半径
        map: np.ndarray 地图，-1为未知，0为空闲，100为障碍物
        connectivity: set 连通性集合
        n_jobs: int 并行数
        """
        self.nbrs = NearestNeighbors(radius=radius, n_jobs=n_jobs)
        self.radius = radius
        self.map = map
        self.connectivity = connectivity_cache

    def fit(self, pos: np.ndarray):
        """
        训练模型

        Parameters
        ----------
        pos: np.ndarray 位置
        """
        self.X = pos
        self.nbrs.fit(pos)
        return self

    def radius_neighbors(
        self,
        point: np.ndarray,
        idx: Optional[int] = None,
        radius: Optional[int] = None,
    ):
        """
        在半径范围内找邻居

        Parameters
        ----------
        point: np.ndarray 点的位置
        idx: int | None 点的索引，None表示不是种子点，不会使用连通性缓存
        radius: int | None 半径，None表示使用bandwidth

        Returns
        -------
        list: 邻居的索引列表
        """
        unconstrained_neighbors = self.nbrs.radius_neighbors(
            [point], radius, return_distance=False, sort_results=False
        )[0]
        result = []
        for neighbor in unconstrained_neighbors:
            if idx is not None:
                if self.connectivity.points_connectable(self.X[idx], self.X[neighbor]):
                    result.append(neighbor)
            else:
                if self.connectivity.points_connectable(self.X[neighbor], point):
                    result.append(neighbor)
            # result.append(neighbor)
        return result

    def get_connectivity(self, idx1: int, idx2: int):
        """
        获取两点之间的连通性

        Parameters
        ----------
        idx1: int 点1的索引
        idx2: int 点2的索引

        Returns
        -------
        bool: 两点之间是否连通
        """
        if self.connectivity.samples_connectable(idx1, idx2):
            return True
        else:
            return False


def mean_shift(
    X,
    map,
    connectivity_cache: ConnectivityCache,
    bandwidth=None,
    seeds=None,
    bin_seeding=False,
    min_bin_freq=1,
    n_jobs=None,
    max_iter=300,
):
    """
    均值漂移聚类

    Parameters
    ----------
    X : np.ndarray
        样本
    map : np.ndarray
        地图，-1为未知，0为空闲，100为障碍物
    connectivity : set
        连通性集合
    bandwidth : float | None
        带宽
    seeds : np.ndarray | None
        种子
    bin_seeding : bool
        是否使用二进制种子
    min_bin_freq : int
        最少的种子数目
    n_jobs : int | None
        并行数
    max_iter : int
        最大迭代次数

    Returns
    -------
    tuple :
        聚类中心，标签
    """
    # 没有定义 bandwidth 时使用 `estimate_bandwidth` 估计带宽
    if bandwidth is None:
        bandwidth = estimate_bandwidth(X, n_jobs=n_jobs)

    # 如果没有设置种子
    if seeds is None:
        # 通过get_bin_seeds选取种子
        # min_bin_freq指定最少的种子数目
        if bin_seeding:
            seeds = get_bin_seeds(X, bandwidth, min_bin_freq)
        # 把所有点设为种子
        else:
            seeds = X
    # 根据shape得到样本数量和特征数量
    n_samples, n_features = X.shape
    # 中心强度字典 键为点 值为强度
    center_intensity_dict = {}

    # 近邻搜索 fit的返回值为
    # radius意思是半径 表示参数空间的范围
    # 用作于radius_neighbors 可以理解为在半径范围内找邻居
    # 使用 n_jobs=1，是因为它将在_mean_shift_single_seed 的并行调用下的嵌套调用中使用，因此无需进一步并行。
    nbrs1 = MyNearestNeighbors(
        radius=bandwidth, map=map, connectivity_cache=connectivity_cache, n_jobs=1
    ).fit(X)

    # 并行地在所有种子上执行迭代
    all_res = Parallel(n_jobs=n_jobs)(
        delayed(_mean_shift_single_seed)(
            (idx, seed), X, nbrs1, map, max_iter, connectivity_cache
        )
        for (idx, seed) in enumerate(seeds)
    )
    """all_res为所有种子迭代完的中心以及邻居数量"""
    # copy results in a dictionary
    for i in range(len(seeds)):
        if all_res[i][1]:
            center_intensity_dict[all_res[i][0]] = all_res[i][1]

    if not center_intensity_dict:
        # nothing near seeds
        raise ValueError(
            "No point was within bandwidth=%f of any seed. Try a different seeding"
            " strategy                              or increase the bandwidth."
            % bandwidth
        )

    sorted_by_intensity = sorted(
        center_intensity_dict.items(),
        key=lambda tup: (tup[1], tup[0]),
        reverse=True,
    )
    sorted_centers = np.array([tup[0] for tup in sorted_by_intensity])
    unique = np.ones(len(sorted_centers), dtype=bool)
    nbrs = MyNearestNeighbors(
        radius=bandwidth, map=map, connectivity_cache=connectivity_cache, n_jobs=n_jobs
    ).fit(sorted_centers)
    for i, center in enumerate(sorted_centers):
        if unique[i]:
            neighbor_idxs = nbrs.radius_neighbors(center)
            unique[neighbor_idxs] = 0
            unique[i] = 1  # leave the current point as unique
    cluster_centers = sorted_centers[unique]

    # ASSIGN LABELS: a point belongs to the cluster that it is closest to
    nbrs = NearestNeighbors(
        n_neighbors=5 if len(cluster_centers) > 5 else len(cluster_centers),
        n_jobs=n_jobs,
    ).fit(cluster_centers)
    labels = np.zeros(n_samples, dtype=int)
    distances, idxs = nbrs.kneighbors(X)
    # labels = idxs.flatten()
    for i, idx in enumerate(idxs):
        labels[i] = idx[0]
        for idx_ in idx:
            if connectivity_cache.points_connectable(X[i], cluster_centers[idx_]):
                labels[i] = idx_
                break

    return cluster_centers, labels


# separate function for each seed's iterative loop
def _mean_shift_single_seed(
    my_mean: np.ndarray,
    X: np.ndarray,
    nbrs: MyNearestNeighbors,
    map: np.ndarray,
    max_iter: int,
    connectivity_cache: ConnectivityCache,
):
    """
    计算单个种子的均值漂移

    Parameters
    ----------
    my_mean: np.ndarray 种子的坐标
    X: np.ndarray 所有点的坐标
    nbrs: MyNearestNeighbors 近邻搜索
    map: np.ndarray 地图，-1为未知，0为空闲，100为障碍物
    max_iter: int 最大迭代次数

    Returns
    -------
    tuple: 种子的均值，邻居的数量，完成的迭代次数
    """
    idx, my_mean = my_mean
    """idx为种子的索引，my_mean为种子的坐标"""
    # 对于每个种子，爬升梯度直到收敛或达到最大迭代次数
    bandwidth = nbrs.radius
    """bandwidth为半径"""
    stop_thresh = 1e-3 * bandwidth
    """stop_thresh为停止阈值"""
    completed_iterations = 0
    """completed_iterations为完成的迭代次数"""
    old_points_within = [my_mean]
    while True:
        # 通过半径找到范围内的点
        i_nbrs = nbrs.radius_neighbors(
            point=my_mean,
            idx=idx if completed_iterations == 0 else None,
            radius=bandwidth,
        )
        points_within = X[i_nbrs]
        """范围内的点"""
        # 如果没有点，结束
        if len(points_within) == 0:
            break
        # break
        my_old_mean = my_mean
        # 计算新的均值
        my_mean = np.mean(points_within, axis=0)
        # 如果新的均值为障碍物或漂移过程中经过障碍物，结束
        if map[
            int(my_mean[0]), int(my_mean[1])
        ] == 100 or not connectivity_cache.points_connectable(my_old_mean, my_mean):
            my_mean = my_old_mean
            points_within = old_points_within
            break
        # 如果新的均值和旧的均值的距离小于停止阈值，或者达到最大迭代次数，结束
        if (
            np.linalg.norm(my_mean - my_old_mean) < stop_thresh
            or completed_iterations == max_iter
        ):
            break
        completed_iterations += 1
        old_points_within = points_within
    return tuple(my_mean), len(points_within), completed_iterations
