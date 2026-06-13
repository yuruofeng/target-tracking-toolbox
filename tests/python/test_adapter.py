from target_tracking_dl import predict


def test_predict_returns_tracking_result_shape():
    result = predict([[1.0, 2.0]], {"name": "smoke"})

    assert set(result) == {
        "metadata",
        "truth",
        "measurements",
        "estimates",
        "metrics",
        "config",
    }
    assert result["metadata"]["backend"] == "python"
    assert result["measurements"] == [[1.0, 2.0]]
    assert result["config"]["name"] == "smoke"
